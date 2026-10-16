# PID Orientation Lab

Aplikasi desktop untuk simulasi dan kendali **yaw satu sumbu** menggunakan servo dan MPU6050. PID dihitung di Python; Arduino membaca IMU dan menerima perintah posisi servo. Visualisasi menampilkan roll, pitch, dan yaw, sedangkan pengendali hanya memakai yaw.

Branch `main` berisi aplikasi dan firmware yang diperlukan untuk menjalankannya. Kode kajian tuning, pengujian, generator laporan, PDF, grafik, dan data eksperimen tersedia di [branch devel](https://github.com/therafaeliger/pmsk-pid-project/tree/devel).

## Instalasi dan menjalankan aplikasi

Gunakan Python 3.10+ dengan Tkinter. Aplikasi telah diuji menggunakan Python 3.14 di Windows.

```powershell
git clone https://github.com/therafaeliger/pmsk-pid-project.git
cd pmsk-pid-project
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe app.py
```

Perintah di atas tidak memerlukan aktivasi virtual environment. Pada Linux/macOS gunakan `.venv/bin/python` sebagai pengganti `.venv\Scripts\python.exe`. Tkinter tersedia pada installer resmi Python Windows; Linux mungkin memerlukan paket `python3-tk`.

Dependensi aplikasi hanya `numpy`, `matplotlib`, dan `pyserial`. Mode **Simulation** dapat dijalankan tanpa hardware.

## Menggunakan simulasi

1. Pilih **Simulation** dan sinyal **Step**, **Impulse**, **Sine**, atau **Manual**.
2. Untuk percobaan pertama, gunakan Kp = 2, Ki = 5, Kd = 0.12, target = 20°, dan batas output = 60°.
3. Klik **Jalankan**. Jika parameter diubah, klik Jalankan untuk menerapkan konfigurasi dan memulai run baru. **Terapkan & reset** menyiapkan run tanpa langsung menjalankannya.
4. **Jeda** menghentikan sementara; Jalankan melanjutkan jika parameter tetap sama. Sesudah run selesai, Jalankan memulai run baru.
5. Pada **Manual**, klik atau seret dial untuk mengubah target yaw antara -60° dan +60°.
6. **Simpan sebagai pembanding** mempertahankan kurva run untuk dibandingkan dengan pengaturan lain.
7. **Ekspor CSV + PNG** menyimpan data, grafik, dan konfigurasi JSON ke lokasi yang dipilih. Pada Arduino, ekspor juga menyertakan berkas `_imu.csv` untuk seluruh telemetry.

Step dimulai pada 0.5 s. Impulse adalah pulsa berdurasi 0.1 s mulai 0.5 s. Sine berfrekuensi 0.5 Hz mulai 0.5 s. Parameter noise, gangguan, frekuensi natural, dan redaman hanya berlaku pada simulasi.

Plant simulasi adalah model posisi orde dua dengan parameter nominal wn = 10 rad/s dan zeta = 0.7. Model ini belum merupakan hasil identifikasi hardware. PID menggunakan derivative-on-measurement terfilter, saturasi output, dan anti-windup. Panel Performa menampilkan gain yang aktif; gain besar dapat memberikan respons serupa saat output sama-sama tersaturasi.

## Hardware dan firmware Arduino

Siapkan Arduino Uno/Nano, servo posisi (misalnya SG90), breakout MPU6050, dan catu servo yang sesuai. Buka `arduino/pid_orientation/pid_orientation.ino` di Arduino IDE, pilih board serta port, lalu upload. Firmware memakai library `Wire` dan `Servo`; tidak memerlukan library MPU6050 pihak ketiga. Jika Servo belum tersedia, instal **Servo by Arduino** melalui Library Manager.

| Perangkat | Koneksi Uno/Nano ATmega328P |
|---|---|
| Sinyal servo | D9 |
| V+ servo | Catu eksternal sesuai spesifikasi servo |
| GND servo | Ground catu dan Arduino bersama |
| MPU6050 SDA | A4 |
| MPU6050 SCL | A5 |
| MPU6050 AD0 | GND, alamat I2C 0x68 |
| VCC dan level I2C MPU6050 | Sesuaikan spesifikasi breakout; chip MPU6050 bukan logika 5 V |

Pasang MPU6050 pada bagian yang bergerak, dengan sumbu Z sejajar poros servo vertikal. Posisi servo 90° menjadi netral mekanis. Pastikan dudukan diam pada netral saat boot untuk kalibrasi gyro; yaw kemudian diukur relatif terhadap posisi awal.

1. Upload firmware, lalu tutup Serial Monitor agar port tersedia untuk aplikasi.
2. Pilih **Arduino**, isi port seperti `COM3`, lalu klik **Hubungkan / putuskan**. Tunggu sekitar 3 detik untuk reset dan kalibrasi.
3. Mode Arduino memakai target **Manual** melalui dial. Sampling diatur ke 0.02 s dan batas output awal 10°.
4. Mulai dengan target dan gain kecil. Periksa bahwa perintah positif menghasilkan yaw positif. Jika arah terbalik, ubah `SERVO_SIGN` menjadi `-1` pada firmware dan upload ulang.
5. Klik **Jalankan**. Preview roll/pitch/yaw tetap diperbarui sebelum run dan selama jeda; PID hanya mengendalikan yaw.
6. **Jeda** mengirim posisi netral. Menjalankan kembali memulai eksperimen hardware baru. Putuskan koneksi saat selesai.

Firmware mengembalikan servo ke netral jika tidak ada perintah selama 300 ms. Aplikasi menghentikan run jika telemetry hilang lebih dari 0.5 s. Netral adalah perintah posisi 90°, bukan mematikan servo. Rentang mekanis aman harus disesuaikan dengan dudukan; batas angka servo tidak menjamin linkage bebas benturan.

MPU6050 tidak memiliki referensi heading absolut, sehingga yaw relatif dapat drift. Boot ulang pada posisi netral untuk kalibrasi ulang. Filter orientasi menggunakan quaternion dengan koreksi gravitasi untuk tilt; roll/pitch ditampilkan sebagai informasi orientasi.

## Protokol serial

Kecepatan 115200 baud, baris ASCII diakhiri newline:

```text
PC -> Arduino: SET,12.500
Arduino -> PC: IMU,1500,10.000,-5.000,12.300
                      ms,roll,pitch,yaw_relative_deg
```

Telemetry dikirim sekitar 100 Hz. Aplikasi memakai timestamp Arduino untuk interval kendali aktual. Firmware lama dengan format `Y,ms,yaw` atau `T,ms,pitch,roll` tidak kompatibel; upload firmware yang disertakan. USB dan antarmuka desktop bukan sistem real-time deterministik.

## Struktur main

```text
app.py                          Antarmuka desktop dan integrasi kendali
pidlab/core.py                  PID, plant simulasi, referensi, dan metrik
pidlab/hardware.py              Koneksi serial dan parser telemetry
pidlab/orientation.py           Transformasi orientasi untuk visualisasi
arduino/pid_orientation/        Firmware MPU6050 dan servo
requirements.txt               Dependensi aplikasi
```

Pengujian numerik/protokol, GUI dengan hardware tiruan, dan kompilasi C++ host firmware tersedia pada `devel`. Build target board Arduino dan pengujian perangkat fisik belum diverifikasi. Lisensi mengikuti [LICENSE](LICENSE).
