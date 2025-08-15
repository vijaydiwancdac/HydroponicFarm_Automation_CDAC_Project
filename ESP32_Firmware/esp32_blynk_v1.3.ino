/* Version: esp32_blynk_v1.3
- Integrated Blynk cloud platform for monitoring hydroponic sensor data
- ESP32 receives STM32 sensor values over UART (temp, humidity, TDS, water level)
- Thresholds updated via Blynk virtual pins (V3/V6/V7), sent back to STM32
- Alert messages auto-triggered using Blynk.logEvent when thresholds are breached
- Includes nutrient dosing calculation when TDS drops below acceptable level
- User-friendly dashboard output via Blynk virtual pins (V0–V5), plus summary on V4
- Serial logs include status updates and diagnostics for easier debugging */

// Blynk Config
#define BLYNK_TEMPLATE_ID "TMPL36mJcVg9w"
#define BLYNK_TEMPLATE_NAME "Hydroponic Farm Dashboard"
#define BLYNK_AUTH_TOKEN "H37uGturZZjeLsBgQwEm_YD-Aq3EOIjk"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Wi-Fi Credentials
char ssid[] = "realme";
char pass[] = "Sangita#2";

// UART Configuration
#define STM_RX 16
#define STM_TX 17
HardwareSerial SerialSTM(2);

// Sensor data variables
int temperature = 0;
int humidity = 0;
int tds = 0;
int waterLevel = 0;

// Thresholds
int tempThreshold = 34;        // Updated via V6
int humidityThreshold = 100;   // Updated via V7
int tdsThreshold = 300;        // Updated via V3
int waterThreshold = 15;

BlynkTimer timer;

// Helper function to send all thresholds to STM32
void sendAllThresholdsToSTM() {
  SerialSTM.printf("%.2f,%.2f,%.2f\n", 
                   static_cast<float>(tempThreshold), 
                   static_cast<float>(humidityThreshold), 
                   static_cast<float>(tdsThreshold));
  Serial.println("📤 Sent all thresholds to STM32:");
  Serial.printf("%.2f,%.2f,%.2f\n",
                static_cast<float>(tempThreshold),
                static_cast<float>(humidityThreshold),
                static_cast<float>(tdsThreshold));
}

// Receive updated thresholds from Blynk app
BLYNK_WRITE(V3)  // TDS
{
  tdsThreshold = param.asInt();
  Serial.printf("TDS Threshold updated (V3): %d\n", tdsThreshold);
  sendAllThresholdsToSTM();
}

BLYNK_WRITE(V6)  // Temperature
{
  tempThreshold = param.asInt();
  Serial.printf("Temperature Threshold updated (V6): %d\n", tempThreshold);
  sendAllThresholdsToSTM();
}

BLYNK_WRITE(V7)  // Humidity
{
  humidityThreshold = param.asInt();
  Serial.printf("Humidity Threshold updated (V7): %d\n", humidityThreshold);
  sendAllThresholdsToSTM();
}


// Send sensor data to Blynk and handle alerts
void sendSensorData()
{
  if (temperature == 0 && humidity == 0 && tds == 0 && waterLevel == 0) {
    Serial.println("⚠ No sensor data from STM32 yet.");
    return;
  }

  Serial.println("---------------");
  Serial.printf("🌡 Temp: %d °C\n", temperature);
  Serial.printf("💧 Humidity: %d %%\n", humidity);
  Serial.printf("🧪 TDS: %d ppm\n", tds);
  Serial.printf("📏 Water Height: %d cm\n", waterLevel);
  Serial.printf("📊 Thresholds - Temp: %d, Humidity: %d, TDS: %d\n",
                tempThreshold, humidityThreshold, tdsThreshold);

  // Convert water height to percentage (Tank height = 21 cm)
  int waterPercent = (waterLevel * 100) / 21;
  float currentWaterVolume = (waterLevel / 21.0) * 4.116;  // in liters

  // Send values to Blynk
  Blynk.virtualWrite(V0, temperature);
  Blynk.virtualWrite(V1, humidity);
  Blynk.virtualWrite(V2, tds);
  Blynk.virtualWrite(V5, waterPercent);  // Use percentage, not raw height

  String message = "";

  if (temperature > tempThreshold) {
    String msg = "🔥 Temp too high: " + String(temperature) + " °C";
    Blynk.logEvent("temperature_alert", msg);
    message += msg + "\n";
    Serial.println(msg);
  }

  if (humidity > humidityThreshold) {
    String msg = "💧 Humidity too high: " + String(humidity) + " %";
    Blynk.logEvent("humidity_alert", msg);
    message += msg + "\n";
    Serial.println(msg);
  }

  if (waterPercent < waterThreshold) {
    String msg = "🚱 Low water level: " + String(waterPercent) + " %";
    Blynk.logEvent("water_level_alert", msg);
    message += msg + "\n";
    Serial.println(msg);
  }

  if (tds < tdsThreshold) {
    float ppmDiff = tdsThreshold - tds;

    // Each 1 mL of nutrient increases 6 ppm per liter
    float nutrientML = (ppmDiff * currentWaterVolume) / 6.0;

    String msg = "🌿 TDS low (" + String(tds) + " ppm).\nAdd " + String(nutrientML, 1) + " mL nutrient.";
    Blynk.logEvent("tds_alert", msg);
    message += msg + "\n";
    Serial.println(msg);
  }

  if (message == "") message = "✅ All values normal.";
  Blynk.virtualWrite(V4, message);
  Serial.println("📨 Summary:");
  Serial.println(message);
}


// Setup
void setup()
{
  Serial.begin(115200);
  SerialSTM.begin(115200, SERIAL_8N1, STM_RX, STM_TX);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(5000L, sendSensorData);
  Serial.println("📡 ESP32 is ready!");
}

// Main loop
void loop()
{
  Blynk.run();
  timer.run();

  if (SerialSTM.available()) {
    String line = SerialSTM.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      float tempF = 0.0, humF = 0.0, tdsF = 0.0, waterF = 0.0;
      int res = sscanf(line.c_str(), "%f,%f,%f,%f", &tempF, &humF, &tdsF, &waterF);

      if (res == 4) {
        temperature = static_cast<int>(tempF);
        humidity = static_cast<int>(humF);
        tds = static_cast<int>(tdsF);
        waterLevel = static_cast<int>(waterF);

        Serial.println("✅ Data received from STM32:");
        Serial.printf("🌡 %d°C | 💧 %d%% | 🧪 %d ppm | 🌊 %d%%\n",
                      temperature, humidity, tds, waterLevel);
      } else {
        Serial.println("❌ Parsing error:");
        Serial.println(line);
      }
    }
  }
}
