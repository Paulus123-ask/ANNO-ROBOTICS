"""
Kontrol PLC FX3U lewat Modbus RTU (RS485) - VERSI RASPBERRY PI (v3, + KAMERA).
Kompatibel dengan pymodbus 3.6.9 (parameter 'slave') maupun versi baru ('device_id').

Sesuai ladder:
    M8000 -> MOV H4083 D8120   (9600, 8 bit, parity Odd, 1 stop bit)
    M8000 -> MOV K1    D8121   (station 1)
    M0..M7 -> Y0..Y7 dengan interlock (Maju/Mundur, Kiri/Kanan, dst.)
    M0 -> MOV K200 D100        (tes: saat M0 ON, D100 berisi 200)

Kamera (mode AUTO, 3 warna):
    merah  -> M0 (Maju)
    putih  -> M1 (Mundur)
    kuning -> M2 (Kiri)
    tidak ada warna -> perintah kamera dilepas

Persiapan di Raspberry Pi:
    python3 -m pip install "pymodbus==3.6.9" pyserial
    sudo apt install python3-opencv
    sudo usermod -a -G dialout $USER     (lalu logout/login atau reboot)
"""

import glob
import inspect
import os
import threading
import time

from pymodbus.client import ModbusSerialClient

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

# ======================= PENGATURAN =======================
PORT = "/dev/ttyUSB0"  # converter USB-RS485; bila tidak ada, dicari otomatis
BAUDRATE = 9600
PARITY = "O"           # Odd, sesuai D8120 = H4083
STOPBITS = 1
BYTESIZE = 8
SLAVE_ID = 1           # sesuai D8121 = K1
DURASI_DEFAULT = 1.0   # detik

REG_D100 = 100         # alamat holding register untuk D100 (cek manual bila beda)

# nomor menu: (nama, coil M, relay Y yang dituju)
GERAKAN = {
    "1": ("Maju", 0, "Y000"),
    "2": ("Mundur", 1, "Y001"),
    "3": ("Kiri", 2, "Y002"),
    "4": ("Kanan", 3, "Y003"),
    "5": ("Naik", 4, "Y004"),
    "6": ("Turun", 5, "Y005"),
    "7": ("Gripper Tutup", 6, "Y006"),
    "8": ("Gripper Buka", 7, "Y007"),
}
# Pasangan arah berlawanan (sama dengan interlock di ladder)
LAWAN = {0: 1, 1: 0, 2: 3, 3: 2, 4: 5, 5: 4, 6: 7, 7: 6}

# ----- Pengaturan kamera -----
CAM_INDEX = 0                      # /dev/video0
CAM_LEBAR, CAM_TINGGI = 640, 480
WARNA_KE_M = {"merah": 0, "putih": 1, "kuning": 2}   # warna -> coil M
MIN_RASIO = 0.05                   # warna harus menutupi >= 5% area ROI
STABIL_FRAME = 5                   # hasil harus sama 5 frame berturut-turut
ROI = (0.25, 0.25, 0.75, 0.75)     # area deteksi (x1, y1, x2, y2) fraksi frame

# Rentang HSV (OpenCV: H 0-179, S 0-255, V 0-255). Sesuaikan dengan cahaya Anda.
RENTANG = {
    "merah":  [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))],
    "kuning": [((20, 100, 100), (35, 255, 255))],
    "putih":  [((0, 0, 200), (179, 40, 255))],
}
WARNA_BGR = {"merah": (0, 0, 255), "kuning": (0, 255, 255), "putih": (255, 255, 255)}
# ==========================================================


def cari_port():
    """Pakai PORT bila ada; jika tidak, cari /dev/ttyUSB* atau /dev/ttyACM*."""
    if os.path.exists(PORT):
        return PORT
    kandidat = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if kandidat:
        print(f"[i] {PORT} tidak ada, memakai {kandidat[0]}")
        return kandidat[0]
    return PORT


PORT_AKTIF = cari_port()

client = ModbusSerialClient(
    port=PORT_AKTIF,
    baudrate=BAUDRATE,
    parity=PARITY,
    stopbits=STOPBITS,
    bytesize=BYTESIZE,
    timeout=1,
)

# Satu port serial dipakai menu (thread utama) dan kamera (thread lain)
plc_lock = threading.Lock()
auto_aktif = threading.Event()     # ON = kamera yang mengendalikan PLC
stop_event = threading.Event()     # sinyal berhenti untuk thread kamera
_perintah_aktif = None             # nomor M yang sedang ON oleh kamera


def panggil(fungsi, *args, **kwargs):
    """Pilih 'device_id' (pymodbus baru) atau 'slave' (pymodbus lama) otomatis."""
    params = inspect.signature(fungsi).parameters
    if "device_id" in params:
        return fungsi(*args, device_id=SLAVE_ID, **kwargs)
    return fungsi(*args, slave=SLAVE_ID, **kwargs)


def tulis_coil(alamat, nilai):
    """Tulis satu coil (M). Mengembalikan True bila berhasil."""
    try:
        with plc_lock:
            res = panggil(client.write_coil, alamat, nilai)
    except Exception as e:
        print(f"[!] Komunikasi gagal (M{alamat}): {e}")
        return False
    if res.isError():
        print(f"[!] PLC tidak menjawab / menolak M{alamat}: {res}")
        return False
    return True


def baca_register(alamat):
    """Baca satu holding register (D). Mengembalikan nilai atau None."""
    try:
        with plc_lock:
            res = panggil(client.read_holding_registers, alamat, count=1)
    except Exception as e:
        print(f"[!] Komunikasi gagal (D{alamat}): {e}")
        return None
    if res.isError():
        print(f"[!] PLC tidak menjawab / menolak baca D{alamat}: {res}")
        return None
    return res.registers[0]


def stop_semua():
    """Matikan semua perintah M0 sampai M7."""
    global _perintah_aktif
    for alamat in range(8):
        tulis_coil(alamat, False)
    _perintah_aktif = None
    print(">> STOP: semua M0-M7 dimatikan")


def gerak(nama, alamat, relay, durasi):
    """Aktifkan satu gerakan selama 'durasi' detik lalu matikan."""
    tulis_coil(LAWAN[alamat], False)    # pastikan arah lawan mati
    if not tulis_coil(alamat, True):
        print(">> Perintah gagal dikirim.")
        return
    print(f">> {nama} ON (M{alamat} -> {relay}) selama {durasi} detik")
    try:
        time.sleep(durasi)
    finally:
        tulis_coil(alamat, False)
        print(f">> {nama} OFF")


def tes_d100():
    """Tes ladder: saat M0 ON, PLC mengisi D100 dengan 200."""
    print(">> Tes: nyalakan M0, baca D100 (harus 200)")
    if not tulis_coil(0, True):
        return
    time.sleep(0.5)
    nilai = baca_register(REG_D100)
    tulis_coil(0, False)
    if nilai is None:
        print("   Gagal membaca D100. Cek alamat register di manual FX3U.")
    elif nilai == 200:
        print("   D100 = 200  -> ladder dan komunikasi OK")
    else:
        print(f"   D100 = {nilai} (diharapkan 200)")


# ======================= KAMERA =======================
def deteksi_warna(frame):
    """Kembalikan (nama_warna / None, rasio per warna). Gambar kotak ROI di frame."""
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = int(ROI[0] * w), int(ROI[1] * h), int(ROI[2] * w), int(ROI[3] * h)
    hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (7, 7), 0)

    rasio = {}
    for nama, daftar in RENTANG.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in daftar:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        rasio[nama] = cv2.countNonZero(mask) / mask.size

    terbaik = max(rasio, key=rasio.get)
    hasil = terbaik if rasio[terbaik] >= MIN_RASIO else None
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return hasil, rasio


def set_perintah(alamat):
    """Matikan perintah kamera lama + lawannya, lalu nyalakan yang baru."""
    global _perintah_aktif
    if alamat == _perintah_aktif:
        return
    if _perintah_aktif is not None:
        tulis_coil(_perintah_aktif, False)
    tulis_coil(LAWAN[alamat], False)
    if tulis_coil(alamat, True):
        _perintah_aktif = alamat
        print(f">> AUTO: M{alamat} ON ({GERAKAN[str(alamat + 1)][0]})")


def lepas_perintah():
    """Matikan perintah kamera yang sedang aktif (tanpa menyalakan yang lain)."""
    global _perintah_aktif
    if _perintah_aktif is not None:
        tulis_coil(_perintah_aktif, False)
        print(f">> AUTO: M{_perintah_aktif} OFF (tidak ada warna)")
        _perintah_aktif = None


def kamera_loop():
    """Tampilkan kamera; bila AUTO ON, warna terdeteksi -> perintah ke PLC."""
    if cv2 is None:
        print("[!] OpenCV belum terpasang (sudo apt install python3-opencv)")
        return
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_LEBAR)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_TINGGI)
    if not cap.isOpened():
        print("[!] Kamera tidak bisa dibuka (cek: ls /dev/video*)")
        return
    print("Kamera terbuka. Tekan 'q' di jendela kamera untuk menutup kamera.")

    terakhir, hitung = None, 0
    try:
        while not stop_event.is_set():
            ok, frame = cap.read()
            if not ok:
                print("[!] Gagal membaca frame")
                break

            hasil, _ = deteksi_warna(frame)
            teks = hasil if hasil else "tidak ada objek"
            cv2.putText(frame, f"Deteksi: {teks}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        WARNA_BGR.get(hasil, (0, 255, 0)), 2)
            aktif = auto_aktif.is_set()
            cv2.putText(frame, "AUTO ON" if aktif else "AUTO OFF", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if aktif else (0, 0, 255), 2)

            # stabilisasi: hasil harus sama beberapa frame berturut-turut
            if hasil == terakhir:
                hitung += 1
            else:
                terakhir, hitung = hasil, 1

            if aktif and hitung == STABIL_FRAME:
                if hasil in WARNA_KE_M:
                    set_perintah(WARNA_KE_M[hasil])
                else:
                    lepas_perintah()

            cv2.imshow("Kamera Robot", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except Exception as e:
        print(f"[!] Kamera error: {e}")
    finally:
        if auto_aktif.is_set():
            auto_aktif.clear()
            stop_semua()
            print(">> Kamera tertutup, AUTO dimatikan.")
        cap.release()
        cv2.destroyAllWindows()


def toggle_auto():
    """Nyalakan / matikan mode AUTO kamera."""
    if auto_aktif.is_set():
        auto_aktif.clear()
        stop_semua()
        print(">> AUTO OFF")
    else:
        auto_aktif.set()
        print(">> AUTO ON (kamera mengendalikan PLC)")
# ======================================================


def tampilkan_menu():
    print("\n===== KONTROL ROBOT =====")
    for k, (nama, _, relay) in GERAKAN.items():
        print(f"  {k}. {nama:14s} ({relay})")
    print("  9. Tes D100 (M0 -> D100 = 200)")
    print("  a. AUTO kamera ON/OFF  (merah=Maju, putih=Mundur, kuning=Kiri)")
    print("  0. STOP semua")
    print("  q. Keluar")


def main():
    if not client.connect():
        print("Tidak bisa membuka port serial.")
        print("Cek: converter terpasang (ls /dev/ttyUSB*), user ada di grup")
        print("dialout (sudo usermod -a -G dialout $USER lalu login ulang),")
        print("dan tidak ada program lain yang memakai port yang sama.")
        return
    print(f"Port {PORT_AKTIF} terbuka ({BAUDRATE}, parity {PARITY}, slave {SLAVE_ID}).")

    # kamera berjalan di thread terpisah supaya menu tetap bisa dipakai
    t_kamera = threading.Thread(target=kamera_loop, daemon=True)
    t_kamera.start()

    try:
        while True:
            tampilkan_menu()
            pilihan = input("Pilih: ").strip().lower()

            if pilihan == "q":
                break
            if pilihan == "0":
                auto_aktif.clear()
                stop_semua()
                continue
            if pilihan == "a":
                toggle_auto()
                continue
            if pilihan == "9":
                if auto_aktif.is_set():
                    print("Matikan AUTO dulu (tekan a).")
                    continue
                tes_d100()
                continue
            if pilihan not in GERAKAN:
                print("Pilihan tidak valid.")
                continue
            if auto_aktif.is_set():
                print("Matikan AUTO dulu (tekan a) sebelum kontrol manual.")
                continue

            nama, alamat, relay = GERAKAN[pilihan]
            teks = input(f"Durasi detik [{DURASI_DEFAULT}]: ").strip()
            try:
                durasi = float(teks) if teks else DURASI_DEFAULT
            except ValueError:
                print("Durasi tidak valid.")
                continue
            if not 0 < durasi <= 10:
                print("Durasi harus antara 0 dan 10 detik.")
                continue

            gerak(nama, alamat, relay, durasi)

    except KeyboardInterrupt:
        print("\nDihentikan oleh pengguna.")
    finally:
        stop_event.set()
        auto_aktif.clear()
        t_kamera.join(timeout=2)
        stop_semua()
        client.close()
        print("Koneksi ditutup.")


if __name__ == "__main__":
    main()