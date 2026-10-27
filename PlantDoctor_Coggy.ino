/*
  ============================================================
   The Plant Doctor  |  Automation vs AI  |  CognoSpace
   Coggy (ESP32) BLE field node for The Plant Doctor web app
  ============================================================
   Board   : ESP32 Dev Module (Arduino-ESP32 core 3.x)
   Library : "DHT sensor library" by Adafruit (+ Adafruit Unified Sensor)
   BLE     : built-in ESP32 BLE library (Nordic UART style service)

   WIRING (matches the #define lines below)
   ------------------------------------------------------------
   DHT-11 DATA ............ GPIO 15
   Soil moisture DO ....... GPIO 32   (digital)
   Rain sensor DO ......... GPIO 33   (digital)
   LDR module DO .......... GPIO 12   (digital, see note below)
   LED (field lights) ..... GPIO 23   (via 220 ohm resistor)

   L298N MOTOR DRIVER (water pump)
   IN1 .................... GPIO 13   (direction)
   ENA .................... GPIO 14   (PWM speed, remove ENA jumper)
   IN2 .................... GND       (so IN1 HIGH = pump runs)
   Water pump ............. OUT1 / OUT2
   Pump supply -> L298N 12V terminal, L298N GND -> ESP32 GND.

   NOTE ON GPIO 12: it is an ESP32 boot (strapping) pin. If the LDR
   module holds it HIGH while the board powers up (room is dark), the
   ESP32 may not start. If that happens, move the LDR wire to GPIO 34
   or 35 and change LDR_PIN below.

   DATA SENT TO THE BROWSER (every 1 s, one JSON line)
   {"t":28.4,"h":61,"sd":1,"rn":0,"dk":0,"p":0,"l":0}
     t  = temperature in C      h  = humidity in %
     sd = 1 soil dry, 0 wet     rn = 1 raining, 0 dry
     dk = 1 dark, 0 bright      p  = pump on/off   l = lights on/off

   COMMANDS FROM THE BROWSER (one per line)
   PUMP:1 / PUMP:0      LED:1 / LED:0
   SPEED:<0-255>        pump PWM speed, saved to flash
   SYNC                 send a reading now
  ============================================================
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <DHT.h>

// ---------- Change per table ----------
#define TABLE_ID "T1"                 // device shows as PlantDoctor-T1

// ---------- Sensor + output pins ----------
#define DHT_PIN    15
#define DHT_TYPE   DHT11
#define SOIL_PIN   32
#define RAIN_PIN   33
#define LDR_PIN    12
#define LED_PIN    23

// ---------- Motor driver (L298N) ----------
#define IN1 13
#define ENA 14

// ---------- Module behaviour (flip if a module reads backwards) ----------
#define SOIL_DRY_LEVEL   HIGH
#define RAIN_WET_LEVEL   LOW
#define LDR_DARK_LEVEL   HIGH

#define DEF_PUMP_SPEED 220

// ---------- BLE UUIDs (must match the web page) ----------
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RX_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define TX_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

DHT dht(DHT_PIN, DHT_TYPE);
Preferences prefs;
BLECharacteristic *txChar = nullptr;
bool deviceConnected = false;
bool wasConnected = false;

int pumpState = 0, ledState = 0;
int pumpSpeed = DEF_PUMP_SPEED;

float lastT = NAN, lastH = NAN;
unsigned long lastSend = 0, lastDht = 0;
bool sendNow = false;

String pendingCmd = "";
portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;

// ---------------- BLE send ----------------
void notifyLine(const char *line) {
  Serial.print(line);
  if (deviceConnected && txChar) {
    txChar->setValue((uint8_t *)line, strlen(line));
    txChar->notify();
  }
}

// ---------------- Outputs ----------------
void setPump(int on) {
  pumpState = on ? 1 : 0;
  if (pumpState) {
    digitalWrite(IN1, HIGH);
    ledcWrite(ENA, pumpSpeed);
  } else {
    ledcWrite(ENA, 0);
    digitalWrite(IN1, LOW);
  }
}

void setLed(int on) {
  ledState = on ? 1 : 0;
  digitalWrite(LED_PIN, ledState);
}

// ---------------- Settings ----------------
void loadSettings() {
  prefs.begin("plantdoc", true);
  pumpSpeed = prefs.getInt("speed", DEF_PUMP_SPEED);
  prefs.end();
}

void saveSettings() {
  prefs.begin("plantdoc", false);
  prefs.putInt("speed", pumpSpeed);
  prefs.end();
}

// ---------------- Commands ----------------
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;
  Serial.print("CMD: "); Serial.println(cmd);

  if      (cmd == "PUMP:1") setPump(1);
  else if (cmd == "PUMP:0") setPump(0);
  else if (cmd == "LED:1")  setLed(1);
  else if (cmd == "LED:0")  setLed(0);
  else if (cmd.startsWith("SPEED:")) {
    pumpSpeed = constrain(cmd.substring(6).toInt(), 0, 255);
    saveSettings();
    if (pumpState) ledcWrite(ENA, pumpSpeed);       // apply new speed live
  }
  else if (cmd == "SYNC") { /* reading is sent below */ }

  sendNow = true;
}

// ---------------- BLE callbacks ----------------
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {
    deviceConnected = true;
    Serial.println("Browser connected");
  }
  void onDisconnect(BLEServer *s) override {
    deviceConnected = false;
    Serial.println("Browser disconnected");
  }
};

class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue();          // core 3.x returns Arduino String
    portENTER_CRITICAL(&cmdMux);
    pendingCmd += v;
    portEXIT_CRITICAL(&cmdMux);
  }
};

// ---------------- Readings ----------------
void sendReading() {
  int soilDry = (digitalRead(SOIL_PIN) == SOIL_DRY_LEVEL) ? 1 : 0;
  int raining = (digitalRead(RAIN_PIN) == RAIN_WET_LEVEL) ? 1 : 0;
  int dark    = (digitalRead(LDR_PIN)  == LDR_DARK_LEVEL) ? 1 : 0;

  char tStr[12], hStr[12], buf[128];
  if (isnan(lastT)) strcpy(tStr, "null"); else snprintf(tStr, sizeof(tStr), "%.1f", lastT);
  if (isnan(lastH)) strcpy(hStr, "null"); else snprintf(hStr, sizeof(hStr), "%.0f", lastH);

  snprintf(buf, sizeof(buf),
    "{\"t\":%s,\"h\":%s,\"sd\":%d,\"rn\":%d,\"dk\":%d,\"p\":%d,\"l\":%d}\n",
    tStr, hStr, soilDry, raining, dark, pumpState, ledState);
  notifyLine(buf);
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nThe Plant Doctor field node starting...");

  pinMode(SOIL_PIN, INPUT);
  pinMode(RAIN_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  pinMode(IN1, OUTPUT);
  ledcAttach(ENA, 1000, 8);           // 1 kHz, 8-bit PWM
  loadSettings();
  setPump(0);
  setLed(0);

  dht.begin();

  String name = String("PlantDoctor-") + TABLE_ID;
  BLEDevice::init(name.c_str());
  BLEDevice::setMTU(185);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  BLEService *svc = server->createService(SERVICE_UUID);

  txChar = svc->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());

  BLECharacteristic *rxChar = svc->createCharacteristic(
    RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxChar->setCallbacks(new RxCB());

  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.printf("Advertising as %s | pump speed %d\n", name.c_str(), pumpSpeed);
}

// ---------------- Loop ----------------
void loop() {
  unsigned long now = millis();

  // Commands from the browser (can arrive several at once)
  String data;
  portENTER_CRITICAL(&cmdMux);
  data = pendingCmd;
  pendingCmd = "";
  portEXIT_CRITICAL(&cmdMux);
  if (data.length()) {
    int start = 0;
    for (int i = 0; i <= (int)data.length(); i++) {
      if (i == (int)data.length() || data[i] == '\n') {
        handleCommand(data.substring(start, i));
        start = i + 1;
      }
    }
  }

  // DHT-11 needs at least 2 s between reads
  if (now - lastDht >= 2000) {
    lastDht = now;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t)) lastT = t;
    if (!isnan(h)) lastH = h;
  }

  if (sendNow || now - lastSend >= 1000) {
    sendNow = false;
    lastSend = now;
    sendReading();
  }

  // Stop the pump and lights, then advertise again if the browser disconnects
  if (!deviceConnected && wasConnected) {
    setPump(0);
    setLed(0);
    delay(300);
    BLEDevice::startAdvertising();
    Serial.println("Advertising again (pump and lights off for safety)");
  }
  wasConnected = deviceConnected;

  delay(20);
}
