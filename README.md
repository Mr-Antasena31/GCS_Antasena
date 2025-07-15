# 🌐 GCS_Antasena - Ground Control System LoRa 

Repositori ini berisi dua komponen utama dalam sistem Ground Control Station (GCS) berbasis komunikasi **LoRa** untuk pemantauan suhu dan kelembaban menggunakan sensor **DHT22**. Sistem ini menggunakan:

- 📡 GCS dan Node sensor dengan **LilyGO LoRa ESP32 OLED**
- 🖥️ GCS berbasis **Jupyter Notebook**
- 🔄 Komunikasi 2 arah melalui **LoRa 923 MHz**

---

## 🎯 Tujuan Proyek

Membangun **Ground Control Station** dari jarak jauh berbasis **Internet of Things (IoT)** menggunakan:

- Komunikasi LoRa Point-to-Point
- Visualisasi data real-time pada Ground Control Station
- Komunikasi efisien di daerah terpencil

---

## ⚙️ Perangkat Keras 1

| Komponen            | Deskripsi                                            |
|---------------------|------------------------------------------------------|
| LilyGO LoRa ESP32   | Modul ESP32 + LoRa + OLED (frekuensi 923 MHz)        |
| Sensor DHT22        | Sensor suhu dan kelembaban digital                   |
| Kabel jumper        | Untuk koneksi antar komponen                         |
| Komputer GCS        | Menjalankan Jupyter Notebook (GCS_ALfan.ipynb)       |

---
## ⚙️ Perangkat Keras 2

| Komponen            | Deskripsi                                                      |
|---------------------|----------------------------------------------------------------|
| Mikrokontroler      | Modul ESP32 + LoRa + OLED (frekuensi 923 MHz) (Main Component) |
| LORA                | Modul Lora/Chip Lora untuk komunikasi (Main component)         |
| OLED                | Untuk Informasi sang Developer (opsional)                      |
| SD Card Module      | Untuk Penyimpanan Backup (opsional)                            |
| Sensor DHT22        | Sensor suhu dan kelembaban digital                             |
| Kabel jumper        | Untuk koneksi antar komponen                                   |
| Komputer GCS        | Menjalankan Jupyter Notebook (GCS_ALfan.ipynb)                 |




---

## 🚀 Cara Menjalankan Sistem

### 📡 1. **Upload Arduino Sketch ke LilyGO**
**Langkah-langkah:**
1. Buka `sketch_jul13a.ino` di Arduino IDE
2. Pilih Board: `ESP32 Dev Module`
3. Pilih Port: COM yang sesuai
4. Pastikan library berikut terinstall:
   - `DHT sensor library`
   - `Adafruit Unified Sensor`
   - `LoRa by Sandeep Mistry`
   - `SSD1306 OLED by Adafruit`
5. Upload kode ke board

### 📊 2. **Jalankan GCS di Jupyter Notebook**
**Langkah-langkah:**
1. Pastikan Python dan Jupyter sudah terinstal.
2. Install library Python yang dibutuhkan:
   ```bash
   pip install pyserial matplotlib

