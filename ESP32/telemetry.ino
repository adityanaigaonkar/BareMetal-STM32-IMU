
#define BLYNK_TEMPLATE_ID "TMPL36riuFleJ"
#define BLYNK_TEMPLATE_NAME "cubesat project aem cp"
#define BLYNK_AUTH_TOKEN "297C3LhaSyal3PojkvzuQXcrv1Gs04d2"

// Prints Blynk diagnostic info to your Serial Monitor
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

HardwareSerial MySerial(1);

/* =====================================================
   WIFI SETTINGS
===================================================== */
const char* ssid = "Aditya";
const char* password = "12345678";

/* =====================================================
   GLOBAL DATA
===================================================== */
String incomingLine = "";
float roll  = 0.0;
float pitch = 0.0;

/* =====================================================
   SETUP
===================================================== */
void setup()
{
    Serial.begin(115200);

    /* UART FROM STM32 */
    // RX on Pin 16, TX disabled (-1)
    MySerial.begin(
        9600,
        SERIAL_8N1,
        16,
        -1
    );

    Serial.println();
    Serial.println("CONNECTING TO BLYNK CLOUD...");

    /* BLYNK CONNECT */
    // Blynk.begin handles both the Wi-Fi connection and the Cloud authentication
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

    Serial.println("BLYNK CONNECTED SUCCESSFULLY!");
}

/* =====================================================
   LOOP
===================================================== */
void loop()
{
    // Keeps the connection to the Blynk Cloud alive
    Blynk.run(); 

    // Read the incoming UART data from the STM32
    while(MySerial.available())
    {
        char c = MySerial.read();

        if(c == '\n')
        {
            /*
            EXPECTED FORMAT:
            ROLL: 0.00  PITCH: 0.00
            */
            int rollIndex = incomingLine.indexOf("ROLL:");
            int pitchIndex = incomingLine.indexOf("PITCH:");

            if(rollIndex >= 0 && pitchIndex >= 0)
            {
                // Isolate the numbers
                String rollString = incomingLine.substring(rollIndex + 5, pitchIndex);
                String pitchString = incomingLine.substring(pitchIndex + 6);

                // Convert to floats
                roll = rollString.toFloat();
                pitch = pitchString.toFloat();

                // Print to Local Serial Monitor for debugging
                Serial.print("ROLL: ");
                Serial.print(roll);
                Serial.print("  PITCH: ");
                Serial.println(pitch);

                // ============================================
                // TRANSMIT TO BLYNK DASHBOARD
                // ============================================
                Blynk.virtualWrite(V1, roll);
                Blynk.virtualWrite(V2, pitch);
            }

            // Clear the buffer for the next reading
            incomingLine = ""; 
        }
        else
        {
            incomingLine += c;
        }
    }
}
