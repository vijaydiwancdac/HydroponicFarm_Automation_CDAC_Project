/*Version: esp32_blynk_v1.1
- Configured ESP32 to receive sensor data from STM32 via UART2 (GPIO 16/17).
- Parsed comma-separated values for temperature, humidity, TDS, and water level.
- Mapped each value to respective Blynk virtual pins (V0, V1, V2, V5).
- Added 2-second timer to periodically push sensor data to Blynk cloud.
- Improved debug visibility via serial monitor logs.
- Handles malformed data with UART parsing error log.*/

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
HardwareSerial SerialSTM(2);  // Use UART2

// Sensor data variables
int temperature = 0;
int humidity = 0;
int tds = 0;
int waterLevel = 0;

BlynkTimer timer;

void sendSensorData() {
  Blynk.virtualWrite(V0, temperature);
  Blynk.virtualWrite(V1, humidity);
  Blynk.virtualWrite(V2, tds);
  Blynk.virtualWrite(V5, waterLevel);
}

void setup() {
  Serial.begin(115200);
  SerialSTM.begin(115200, SERIAL_8N1, STM_RX, STM_TX);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("📡 ESP32 UART interface ready");

  timer.setInterval(2000L, sendSensorData);
}

void loop() {
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
        Serial.println("❌ UART parsing error:");
        Serial.println(line);
      }
    }
  }
}
