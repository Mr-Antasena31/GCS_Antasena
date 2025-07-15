#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FS.h>
#include <SD.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>

// --- Konfigurasi Pin LilyGO T-Beam ---

// === OLED Config ===
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// === LoRa Pinout for LilyGO T-Beam (Common Configuration) ===
// Ini adalah pin VSPI default untuk LoRa pada banyak papan ESP32 LilyGO
#define LORA_SCK    5
#define LORA_MISO   19
#define LORA_MOSI   27
#define LORA_CS     18    // Chip Select for LoRa module
#define LORA_RST    23    // Reset pin for LoRa module
#define LORA_IRQ    26    // Interrupt pin for LoRa module (sometimes called DIO0)

#define LORA_FREQ         923E6    // LoRa frequency (e.g., 923MHz for Indonesia)
#define LORA_SF           12       // Spreading Factor (6-12)
#define LORA_BW           250E3    // Signal Bandwidth (e.g., 125KHz, 250KHz, 500KHz)
#define LORA_CR           4        // Coding Rate (5-8)
#define LORA_SYNC_WORD    0x18     // Sync Word (0x12 for public, 0x34 for private, 0x18 for TTGO default)
#define LORA_PREAMBLE_LEN 8        // Preamble Length
#define LORA_TX_POWER     20       // TX Power (up to 20dBm for LoRaWAN, check local regulations)

// === Custom SD CARD PINS for LilyGO T-Beam (Using HSPI) ===
// Pada LilyGO T-Beam, SD card sering terhubung ke HSPI.
// Pastikan pin ini sesuai dengan diagram pinout spesifik board Anda.
#define SD_CS     13    // Chip Select for SD Card (common on T-Beam)
#define SD_CLK    14    // HSPI SCK
#define SD_MISO   2     // HSPI MISO
#define SD_MOSI   15    // HSPI MOSI

SPIClass customSPI(HSPI); // Menggunakan HSPI untuk SD card (agar tidak konflik dengan VSPI LoRa)

// === DHT22 Config ===
#define DHTPIN 21       // Pin Digital yang terhubung ke sensor DHT (misalnya, GPIO21)
#define DHTTYPE DHT22   // Jenis sensor DHT (DHT11, DHT21, DHT22)
DHT dht(DHTPIN, DHTTYPE);

// --- Variabel Global ---
bool autoPing = false;
unsigned long lastPing = 0;
const unsigned long pingInterval = 10000; // Kirim ping setiap 10 detik

String lastData = "-";      // Data LoRa terakhir yang diterima
int lastRssi = 0;           // RSSI dari paket LoRa terakhir yang diterima
int lastTxRssi = 0;         // Placeholder for TX RSSI (tidak langsung tersedia di sisi GCS)

float lastHumidity = 0.0;   // Kelembaban terakhir dari DHT
float lastTemperature = 0.0;// Suhu terakhir dari DHT
uint32_t sdCardTotalBytes = 0; // Total ukuran SD card GCS
uint32_t sdCardUsedBytes = 0;  // Ukuran SD card yang terpakai GCS

unsigned long lastDHTRead = 0;
const unsigned long dhtReadInterval = 30000; // Baca DHT setiap 30 detik

unsigned long lastOLEDUpdate = 0;
const unsigned long oledUpdateInterval = 2000; // Update OLED setiap 2 detik

// --- Variabel untuk UI dan Grafik ---
enum ScreenState {
  SCREEN_MAIN,      // Layar utama dengan data ringkasan
  SCREEN_RSSI_GRAPH, // Layar grafik RSSI
  SCREEN_DHT_GRAPH   // Layar grafik Suhu dan Kelembaban
};
ScreenState currentScreen = SCREEN_MAIN; // Layar awal saat boot

const int MAX_GRAPH_POINTS = 20; // Jumlah titik data yang akan ditampilkan pada grafik
int rssiHistory[MAX_GRAPH_POINTS];
float tempHistory[MAX_GRAPH_POINTS];
float humHistory[MAX_GRAPH_POINTS];
// dataIndex tidak lagi diperlukan karena kita menggeser array

unsigned long lastScreenChange = 0;
const unsigned long screenChangeInterval = 5000; // Ganti layar setiap 5 detik

unsigned long lastSDInfoUpdate = 0;
const unsigned long sdInfoUpdateInterval = 5 * 60 * 1000; // 5 menit dalam milidetik

// --- Prototipe Fungsi ---
void showMenu();
void handleCommand(String cmd);
void sendLoRa(String msg);
void showResponse(String data, int rssi);
void logToSD(String filename, String data);
void readLogFile(String filename);
void deleteLogFile(String filename);
void updateSingleOLEDScreen();
void drawRssiGraph();
void drawDhtGraph();
void readDHTData();
void getSDCardInfo();
void switchScreen();
void displaySDCardInfoOnSerial(); // Menampilkan info SD card di Serial Monitor

// --- Setup ---
void setup() {
  Serial.begin(115200); // Baud rate harus sama dengan di Python jika digunakan
  while (!Serial); // Tunggu Serial Monitor terbuka

  // Inisialisasi OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Alamat umum 0x3C
    Serial.println("❌ OLED Init Failed!");
    // Jangan hentikan program di sini agar fungsi lain tetap bisa diuji
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("GCS ANTASENA");
    display.display();
    Serial.println("✅ OLED Init Success!");
    delay(500);
  }

  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║          Ground Control Station (GCS)  ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println("GCS Siap! Menunggu perintah serial dan data LoRa...");

  // Inisialisasi modul LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); // Menggunakan SPI default (VSPI)
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("❌ LoRa Init Failed!");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("LoRa Init Failed!");
    display.display();
    while (true); // Hentikan jika LoRa gagal inisialisasi (sangat penting)
  }

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.enableCrc();

  Serial.println("✅ LoRa Init Success!");
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("LoRa Init Success!");
  display.display();
  delay(500);

  // Inisialisasi SD Card dengan custom pins (HSPI)
  customSPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS); // Gunakan HSPI untuk SD.

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Init SD Card...");
  display.display();
  delay(500);

  if (!SD.begin(SD_CS, customSPI)) { // Gunakan customSPI untuk SD card
    Serial.println("❌ SD Card Init Failed!");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("SD Init Failed!");
    display.display();
  } else {
    Serial.println("✅ SD Card Init Success!");
    getSDCardInfo(); // Dapatkan info SD card awal
    displaySDCardInfoOnSerial(); // Tampilkan info SD card awal di serial
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("SD Init Success!");
    display.print("SD Card: ");
    display.print(sdCardTotalBytes / (1024 * 1024));
    display.println("MB");
    display.display();
  }
  delay(500);

  // Inisialisasi sensor DHT22
  Serial.println("Init DHT22 Sensor...");
  dht.begin();
  Serial.println("✅ DHT22 Sensor Init Success!");
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("DHT22 Init Success!");
  display.display();
  delay(500);

  // Inisialisasi histori data grafik
  for (int i = 0; i < MAX_GRAPH_POINTS; i++) {
    rssiHistory[i] = 0;
    tempHistory[i] = 0.0;
    humHistory[i] = 0.0;
  }

  showMenu(); // Tampilkan menu perintah awal
  LoRa.receive(); // Atur LoRa ke mode penerimaan setelah setup

  readDHTData(); // Lakukan pembacaan DHT awal
  updateSingleOLEDScreen(); // Update OLED awal
  lastScreenChange = millis(); // Set waktu awal pergantian layar
  lastSDInfoUpdate = millis(); // Set waktu awal update info SD card
}

// --- Loop ---
void loop() {
  // Tangani perintah serial dari pengguna (misal dari aplikasi Python atau Serial Monitor)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // Hapus spasi di awal/akhir
    handleCommand(cmd);
  }

  // Tangani paket LoRa yang masuk
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }
    int rssi = LoRa.packetRssi();
    lastData = incoming;
    lastRssi = rssi;
    showResponse(incoming, rssi); // Tampilkan di serial monitor

    // Simpan RSSI ke history untuk grafik
    for (int i = 0; i < MAX_GRAPH_POINTS - 1; i++) {
      rssiHistory[i] = rssiHistory[i + 1];
    }
    rssiHistory[MAX_GRAPH_POINTS - 1] = rssi;

    String logEntry = "[DATA] " + incoming + " | RSSI: " + String(rssi) + " dBm";
    logToSD("/log.txt", logEntry); // Log data LoRa ke SD card

    // Kirim data LoRa ke Serial Monitor dalam format JSON (untuk parsing mudah oleh GUI)
    String jsonOutput = "{\"type\": \"lora_data\", ";
    jsonOutput += "\"data\": \"" + incoming + "\", ";
    jsonOutput += "\"rssi\": " + String(rssi) + "}";
    Serial.println(jsonOutput);
  }

  // Fungsionalitas auto ping
  if (autoPing && millis() - lastPing >= pingInterval) {
    lastPing = millis();
    sendLoRa("1"); // Kirim perintah '1' (Uptime Picosatelit)
  }

  // Pembacaan DHT22 otomatis setiap dhtReadInterval
  if (millis() - lastDHTRead >= dhtReadInterval) {
    lastDHTRead = millis();
    readDHTData();      // Baca dan update variabel data DHT

    // Simpan data DHT ke history untuk grafik
    for (int i = 0; i < MAX_GRAPH_POINTS - 1; i++) {
      tempHistory[i] = tempHistory[i + 1];
      humHistory[i] = humHistory[i + 1];
    }
    tempHistory[MAX_GRAPH_POINTS - 1] = lastTemperature;
    humHistory[MAX_GRAPH_POINTS - 1] = lastHumidity;

    // Kirim data DHT ke Serial Monitor dalam format JSON
    String jsonOutput = "{\"type\": \"dht_data\", ";
    jsonOutput += "\"temperature\": " + String(lastTemperature, 1) + ", ";
    jsonOutput += "\"humidity\": " + String(lastHumidity, 1) + "}";
    Serial.println(jsonOutput);
  }

  // Update panel OLED secara berkala
  if (millis() - lastOLEDUpdate >= oledUpdateInterval) {
    lastOLEDUpdate = millis();
    getSDCardInfo(); // Selalu refresh info SD card sebelum update layar

    // Panggil fungsi update layar sesuai currentScreen
    switch (currentScreen) {
      case SCREEN_MAIN:
        updateSingleOLEDScreen();
        break;
      case SCREEN_RSSI_GRAPH:
        drawRssiGraph();
        break;
      case SCREEN_DHT_GRAPH:
        drawDhtGraph();
        break;
    }
  }

  // Update info SD Card ke Serial setiap 5 menit
  if (millis() - lastSDInfoUpdate >= sdInfoUpdateInterval) {
    lastSDInfoUpdate = millis();
    displaySDCardInfoOnSerial(); // Panggil fungsi untuk menampilkan info SD
  }

  // Otomatis mengganti layar OLED
  if (millis() - lastScreenChange >= screenChangeInterval) {
    lastScreenChange = millis();
    switchScreen();
  }
}

// --- Fungsi-fungsi Pendukung ---

// Fungsi untuk update layar OLED tunggal
void updateSingleOLEDScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); // Font kecil untuk informasi lebih banyak

  display.setCursor(0, 0);
  display.println("GCS ANTASENA 1"); // Header tetap

  display.setCursor(0, 10);
  display.print("Kirim RSSI: ");
  // lastTxRssi akan tetap 0 kecuali ada ACK dari picosat dengan RSSI.
  display.print(lastTxRssi);
  display.println(" dBm");

  display.setCursor(0, 20);
  display.print("Terima RSSI: ");
  display.print(lastRssi);
  display.println(" dBm");

  display.setCursor(0, 30);
  display.print("Suhu: ");
  if (isnan(lastTemperature)) {
    display.println("N/A *C");
  } else {
    display.print(lastTemperature, 1); // Tampilkan 1 desimal
    display.println(" *C");
  }

  display.setCursor(0, 40);
  display.print("Kelembaban: ");
  if (isnan(lastHumidity)) {
    display.println("N/A %");
  } else {
    display.print(lastHumidity, 1); // Tampilkan 1 desimal
    display.println(" %");
  }

  display.setCursor(0, 50);
  display.print("SD Sisa: ");
  if (SD.cardType() == CARD_NONE) {
    display.println("No SD");
  } else {
    uint32_t freeBytes = sdCardTotalBytes - sdCardUsedBytes;
    display.print(freeBytes / (1024 * 1024)); // Free MB
    display.print("/");
    display.print(sdCardTotalBytes / (1024 * 1024)); // Total MB
    display.println("MB");
  }

  display.display();
}

// Fungsi untuk menggambar grafik RSSI
void drawRssiGraph() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("RSSI Graph (-dBm)");

  // Garis horizontal dan vertikal untuk grid/sumbu
  display.drawFastHLine(0, SCREEN_HEIGHT - 10, SCREEN_WIDTH, SSD1306_WHITE); // Sumbu X
  display.drawFastVLine(10, 10, SCREEN_HEIGHT - 20, SSD1306_WHITE); // Sumbu Y (Disisihkan untuk judul)

  // Skala RSSI (disesuaikan)
  int yAxisMin = -120; // Misal, -120 dBm adalah terburuk
  int yAxisMax = -60;  // Misal, -60 dBm adalah terbaik
  
  // Menggambar titik-titik data
  int xStep = (SCREEN_WIDTH - 20) / (MAX_GRAPH_POINTS - 1); // Lebar grafik - margin / jumlah titik
  for (int i = 0; i < MAX_GRAPH_POINTS - 1; i++) {
    int x1 = 10 + (i * xStep);
    // map(value, fromLow, fromHigh, toLow, toHigh)
    // Tinggi OLED menurun, jadi toLow (bawah layar) adalah nilai terbesar, toHigh (atas layar) adalah nilai terkecil
    int y1 = map(rssiHistory[i], yAxisMin, yAxisMax, SCREEN_HEIGHT - 10, 10);

    int x2 = 10 + ((i + 1) * xStep);
    int y2 = map(rssiHistory[i + 1], yAxisMin, yAxisMax, SCREEN_HEIGHT - 10, 10);

    display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
    display.drawCircle(x1, y1, 1, SSD1306_WHITE); // Gambar titik
  }
  // Gambar titik terakhir
  display.drawCircle(10 + ((MAX_GRAPH_POINTS - 1) * xStep), map(rssiHistory[MAX_GRAPH_POINTS - 1], yAxisMin, yAxisMax, SCREEN_HEIGHT - 10, 10), 1, SSD1306_WHITE);

  // Tampilkan nilai RSSI terakhir di pojok kanan atas
  display.setCursor(SCREEN_WIDTH - 25, 0);
  display.print(lastRssi);
  
  display.display();
}

// Fungsi untuk menggambar grafik DHT (Suhu dan Kelembaban)
void drawDhtGraph() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Temp/Hum Graph");

  // Garis horizontal dan vertikal untuk grid/sumbu
  display.drawFastHLine(0, SCREEN_HEIGHT - 10, SCREEN_WIDTH, SSD1306_WHITE); // Sumbu X
  display.drawFastVLine(10, 10, SCREEN_HEIGHT - 20, SSD1306_WHITE); // Sumbu Y

  // Skala Suhu (disesuaikan)
  float tempMin = 20.0;
  float tempMax = 40.0;
  // Skala Kelembaban (disesuaikan)
  float humMin = 40.0;
  float humMax = 90.0;

  int xStep = (SCREEN_WIDTH - 20) / (MAX_GRAPH_POINTS - 1);

  // Menggambar grafik Suhu (warna putih solid)
  for (int i = 0; i < MAX_GRAPH_POINTS - 1; i++) {
    int x1 = 10 + (i * xStep);
    int y1_temp = map(tempHistory[i] * 10, tempMin * 10, tempMax * 10, SCREEN_HEIGHT - 10, 10); // Kalikan 10 untuk akurasi map float

    int x2 = 10 + ((i + 1) * xStep);
    int y2_temp = map(tempHistory[i + 1] * 10, tempMin * 10, tempMax * 10, SCREEN_HEIGHT - 10, 10);

    display.drawLine(x1, y1_temp, x2, y2_temp, SSD1306_WHITE);
    display.drawCircle(x1, y1_temp, 1, SSD1306_WHITE);
  }
  display.drawCircle(10 + ((MAX_GRAPH_POINTS - 1) * xStep), map(tempHistory[MAX_GRAPH_POINTS - 1] * 10, tempMin * 10, tempMax * 10, SCREEN_HEIGHT - 10, 10), 1, SSD1306_WHITE);


  // Menggambar grafik Kelembaban (gunakan titik-titik untuk membedakan dari Suhu)
  for (int i = 0; i < MAX_GRAPH_POINTS - 1; i++) {
    int x1 = 10 + (i * xStep);
    int y1_hum = map(humHistory[i] * 10, humMin * 10, humMax * 10, SCREEN_HEIGHT - 10, 10);

    int x2 = 10 + ((i + 1) * xStep);
    int y2_hum = map(humHistory[i + 1] * 10, humMin * 10, humMax * 10, SCREEN_HEIGHT - 10, 10);
    
    // Gambar hanya titik-titik atau garis putus-putus pendek
    display.drawPixel(x1, y1_hum, SSD1306_WHITE); 
    display.drawPixel(x1 + 1, y1_hum, SSD1306_WHITE); // Membuat titik lebih tebal
    display.drawPixel(x2, y2_hum, SSD1306_WHITE);
    display.drawPixel(x2 + 1, y2_hum, SSD1306_WHITE); // Membuat titik lebih tebal
  }

  // Tampilkan nilai terakhir di pojok kanan atas (Suhu/Kelembaban)
  display.setCursor(SCREEN_WIDTH - 35, 0); // Sesuaikan posisi agar muat
  display.print(lastTemperature, 0); // Suhu tanpa desimal
  display.print("/");
  display.print(lastHumidity, 0); // Kelembaban tanpa desimal
  
  display.display();
}

// Fungsi untuk mengganti layar secara bergilir
void switchScreen() {
  switch (currentScreen) {
    case SCREEN_MAIN:
      currentScreen = SCREEN_RSSI_GRAPH;
      break;
    case SCREEN_RSSI_GRAPH:
      currentScreen = SCREEN_DHT_GRAPH;
      break;
    case SCREEN_DHT_GRAPH:
      currentScreen = SCREEN_MAIN;
      break;
  }
  // Paksa update layar baru segera setelah pergantian
  lastOLEDUpdate = millis() - oledUpdateInterval - 1; 
}

// Fungsi untuk menangani perintah dari Serial (misal dari aplikasi Python)
void handleCommand(String cmd) {
  Serial.print("Menerima perintah: '");
  Serial.print(cmd);
  Serial.println("'");

  if (cmd == "menu") {
    showMenu();
  } else if (cmd == "auto") {
    autoPing = true;
    lastPing = millis();
    Serial.println("🔁 Auto Ping ENABLED. Mengirim perintah 1 setiap 10 detik.");
    Serial.println("{\"type\": \"info\", \"message\": \"Auto Ping: AKTIF\"}");
  } else if (cmd == "stop") {
    autoPing = false;
    Serial.println("🚫 Auto Ping DISABLED.");
    Serial.println("{\"type\": \"info\", \"message\": \"Auto Ping: NONAKTIF\"}");
  } else if (cmd == "storage") {
    sendLoRa("10"); // Perintah ke picosatellite untuk cek penyimpanan SD card
  } else if (cmd == "files") {
    sendLoRa("11"); // Perintah ke picosatellite untuk daftar file di SD card
  } else if (cmd == "readlog") {
    readLogFile("/log.txt"); // Baca file log GCS lokal
  } else if (cmd == "clearlog") {
    deleteLogFile("/log.txt"); // Hapus file log GCS lokal
  } else if (cmd == "display_main") { // Perintah untuk ganti layar ke main
    currentScreen = SCREEN_MAIN;
    lastScreenChange = millis(); // Reset timer pergantian layar
    updateSingleOLEDScreen();
    Serial.println("{\"type\": \"info\", \"message\": \"Menampilkan Layar Utama\"}");
  } else if (cmd == "display_rssi") { // Perintah untuk ganti layar ke grafik RSSI
    currentScreen = SCREEN_RSSI_GRAPH;
    lastScreenChange = millis();
    drawRssiGraph();
    Serial.println("{\"type\": \"info\", \"message\": \"Menampilkan Grafik RSSI\"}");
  } else if (cmd == "display_dht") { // Perintah untuk ganti layar ke grafik DHT
    currentScreen = SCREEN_DHT_GRAPH;
    lastScreenChange = millis();
    drawDhtGraph();
    Serial.println("{\"type\": \"info\", \"message\": \"Menampilkan Grafik DHT\"}");
  }
  else if (cmd.length() == 1 && isDigit(cmd[0]) && cmd.toInt() >= 1 && cmd.toInt() <= 9) {
    sendLoRa(cmd); // Kirim perintah numerik (1-9) ke picosatellite
  } else {
    Serial.println("❗ Input tidak valid. Ketik 1-9, 'auto', 'stop', 'storage', 'files', 'readlog', 'clearlog', 'display_main', 'display_rssi', 'display_dht', atau 'menu'.");
    Serial.println("{\"type\": \"error\", \"message\": \"Perintah tidak valid\"}");
  }
}

// Fungsi untuk mengirim pesan LoRa
void sendLoRa(String msg) {
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();
  Serial.print("📤 Mengirim Perintah → ");
  Serial.println(msg);
  Serial.println("{\"type\": \"info\", \"message\": \"Mengirim perintah LoRa: " + msg + "\"}");
  delay(100); // Penundaan singkat untuk memastikan paket terkirim
  LoRa.receive(); // Kembali ke mode penerimaan
}

// Fungsi untuk menampilkan data LoRa yang diterima
void showResponse(String data, int rssi) {
  Serial.println("\n📥 Data Diterima dari Picosatellite:");
  Serial.println("╔════════════════════════════════════════════════════════╗");
  Serial.print    ("║ Data : "); Serial.println(data);
  Serial.print    ("║ RSSI : "); Serial.print(rssi); Serial.println(" dBm");
  Serial.println("╚════════════════════════════════════════════════════════╝");
}

// Fungsi untuk membaca data sensor DHT22 lokal
void readDHTData() {
  Serial.println("\n🌡️ Membaca Data DHT22 Lokal...");
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("❌ Gagal membaca dari sensor DHT! Periksa perkabelan dan sensor.");
    lastHumidity = NAN; // Simpan NaN untuk menunjukkan kegagalan
    lastTemperature = NAN; // Simpan NaN
    String logEntry = "[ERROR] Pembacaan DHT Gagal!";
    logToSD("/log.txt", logEntry); // Log error ke SD card
    Serial.println("{\"type\": \"error\", \"message\": \"Pembacaan DHT Gagal!\"}");
    return;
  }
  // Koreksi atau kalibrasi sederhana (jika diperlukan)
  h = h - 5; // Contoh kalibrasi
  t = t - 3; // Contoh kalibrasi

  Serial.print("Kelembaban: ");
  Serial.print(h, 1); // Tampilkan 1 desimal
  Serial.print(" %\t");
  Serial.print("Suhu: ");
  Serial.print(t, 1); // Tampilkan 1 desimal
  Serial.println(" *C");

  lastHumidity = h; // Simpan pembacaan valid terbaru
  lastTemperature = t;

  String logEntry = "[DHT] Kelembaban: " + String(h, 1) + "% | Suhu: " + String(t, 1) + "*C";
  logToSD("/log.txt", logEntry); // Log data DHT ke SD card
}

// Fungsi untuk mendapatkan info SD card dan menyimpannya di variabel global
void getSDCardInfo() {
  if (SD.cardType() == CARD_NONE) {
    sdCardTotalBytes = 0;
    sdCardUsedBytes = 0;
    return;
  }
  sdCardTotalBytes = SD.cardSize(); // Mendapatkan total ukuran kartu
  sdCardUsedBytes = SD.usedBytes(); // Mendapatkan ukuran yang terpakai
}

// Fungsi untuk menampilkan info SD card di Serial Monitor
void displaySDCardInfoOnSerial() {
    getSDCardInfo(); // Selalu dapatkan info terbaru sebelum ditampilkan
    String jsonOutput = "{\"type\": \"sd_info\", ";
    jsonOutput += "\"total_bytes\": " + String(sdCardTotalBytes) + ", ";
    jsonOutput += "\"used_bytes\": " + String(sdCardUsedBytes) + "}";
    Serial.println(jsonOutput); // Output JSON untuk parsing GUI

    Serial.print("ℹ️ SD Card Info: Total ");
    Serial.print(sdCardTotalBytes / (1024 * 1024));
    Serial.print("MB, Digunakan ");
    Serial.print(sdCardUsedBytes / (1024 * 1024));
    Serial.print("MB, Sisa ");
    Serial.print((sdCardTotalBytes - sdCardUsedBytes) / (1024 * 1024));
    Serial.println("MB");
}

// Fungsi untuk menampilkan menu perintah (hanya di Serial Monitor)
void showMenu() {
  Serial.println("\n📋 MENU PERINTAH GCS");
  Serial.println("╔════════╦════════════════════════════╗");
  Serial.println("║   CMD  ║        FUNGSI              ║");
  Serial.println("╠════════╬════════════════════════════╣");
  Serial.println("║    1   ║ Picosatellite Uptime       ║");
  Serial.println("║    2   ║ Data DHT22 Lokal           ║");
  Serial.println("║    3   ║ Info Penyimpanan Internal  ║");
  Serial.println("║    4   ║ Info Kartu SD              ║");
  Serial.println("║    5   ║ Status Daya                ║");
  Serial.println("║    6   ║ Simpan Data ke SD          ║");
  Serial.println("║    7   ║ Hapus Data di SD           ║");
  Serial.println("║    8   ║ Daftar File di SD          ║");
  Serial.println("║    9   ║ 5 Log Terakhir             ║");
  Serial.println("╠════════╩════════════════════════════╣");
  Serial.println("║   auto     → Auto ping CMD 1 setiap 10s║");
  Serial.println("║   stop     → Hentikan Auto Ping       ║");
  Serial.println("║   storage  → Cek Penyimpanan SD Card  ║");
  Serial.println("║   files    → Daftar File di SD Card   ║");
  Serial.println("║   readlog  → Baca log.txt             ║");
  Serial.println("║   clearlog → Hapus log.txt            ║");
  Serial.println("║   display_main → Tampilkan Layar Utama ║");
  Serial.println("║   display_rssi → Tampilkan Grafik RSSI ║");
  Serial.println("║   display_dht  → Tampilkan Grafik DHT  ║");
  Serial.println("║   menu     → Tampilkan Menu ini lagi  ║");
  Serial.println("╚═════════════════════════════════════╝\n");
}

// === SD Card Utilities ===
void logToSD(String filename, String data) {
  // Coba buka file dalam mode APPEND (menambahkan ke akhir file)
  File file = SD.open(filename.c_str(), FILE_APPEND);
  if (file) {
    file.println(data); // Tulis data dan baris baru
    file.close();       // Tutup file
    Serial.println("✅ Data disimpan ke SD: " + filename);
    displaySDCardInfoOnSerial(); // Tampilkan info SD yang diperbarui setelah menulis
  } else {
    // Jika gagal membuka, mungkin file belum ada, coba buat baru (mode WRITE)
    Serial.println("❌ Gagal membuka file untuk menulis! Mencoba membuat file baru.");
    file = SD.open(filename.c_str(), FILE_WRITE); // Coba buat jika tidak ada
    if (file) {
      file.println(data);
      file.close();
      Serial.println("✅ File baru dibuat dan data disimpan.");
      displaySDCardInfoOnSerial(); // Tampilkan info SD yang diperbarui setelah menulis
    } else {
      Serial.println("❌ Gagal membuat file baru.");
      Serial.println("{\"type\": \"error\", \"message\": \"Gagal menulis log ke SD Card!\"}");
    }
  }
}

void readLogFile(String filename) {
  File file = SD.open(filename.c_str());
  if (!file) {
    Serial.println("❌ Gagal membuka file log atau file tidak ditemukan!");
    Serial.println("{\"type\": \"error\", \"message\": \"Log file tidak ditemukan!\"}");
    return;
  }

  Serial.println("\n📖 Isi File " + filename + ":");
  Serial.println("{\"type\": \"info\", \"message\": \"--- Isi File " + filename + ": ---\"}");
  while (file.available()) {
    String line = file.readStringUntil('\n'); // Baca baris per baris
    Serial.println(line); // Tampilkan di Serial Monitor
    Serial.println("{\"type\": \"log_line\", \"content\": \"" + line + "\"}"); // Output JSON
  }
  file.close(); // Tutup file
  Serial.println("\n📚 EOF - Akhir File.\n");
  Serial.println("{\"type\": \"info\", \"message\": \"--- EOF - Akhir File. ---\"}");
}

void deleteLogFile(String filename) {
  if (SD.exists(filename.c_str())) { // Periksa apakah file ada
    SD.remove(filename.c_str());    // Hapus file
    Serial.println("🗑️ File log dihapus: " + filename);
    Serial.println("{\"type\": \"info\", \"message\": \"File log dihapus: " + filename + "\"}");
    displaySDCardInfoOnSerial(); // Tampilkan info SD yang diperbarui setelah menghapus
  } else {
    Serial.println("⚠️ File log tidak ditemukan!");
    Serial.println("{\"type\": \"info\", \"message\": \"File log tidak ditemukan!\"}");
  }
}