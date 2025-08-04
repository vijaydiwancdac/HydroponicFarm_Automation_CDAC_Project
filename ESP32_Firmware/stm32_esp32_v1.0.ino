// UART Configuration
#define STM_RX 16
#define STM_TX 17
HardwareSerial SerialSTM(2);  // Use UART2

// Sensor data variables
int temperature = 0;
int humidity = 0;
int tds = 0;
int waterLevel = 0;

void setup() {
  Serial.begin(115200);
  SerialSTM.begin(115200, SERIAL_8N1, STM_RX, STM_TX);
  Serial.println("📡 ESP32 listening for STM32 data...");
}

void loop() {
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

  delay(1000);  // Optional delay to reduce CPU load
}
