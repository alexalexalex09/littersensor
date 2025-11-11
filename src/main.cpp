#include <Arduino.h>

// This program monitors a PIR motion sensor (for deep sleep wake-up) and an external reboot button on an ESP8266.
// NOTE: For the PIR sensor to wake the ESP8266 from deep sleep, its output MUST be connected to the
// special wake-up pin, typically GPIO16 (D0) on a NodeMCU, which is wired to the RST pin.

// --- Pin Definitions for NodeMCU ESP-12E ---
#define PIR_SENSOR_PIN D0    // GPIO16 - Recommended for ESP8266 deep sleep wake-up (connects to RST)
#define LED_PIN D4           // GPIO2 - Pin for the permanent alert LED
#define REBOOT_BUTTON_PIN D5 // GPIO14 - Pin for the external reboot button

// --- State Persistence in RTC User Memory ---
struct RtcData
{
  int wakeupCounter;
};
RtcData rtcData; // Global structure instance

// The address in RTC memory to store the data structure
const int RTC_ADDR = 64;

void setup()
{
  // Initialize Serial Communication
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n--- ESP8266 Alert System Initialized ---");

  // Load the counter state from RTC memory
  if (ESP.rtcUserMemoryRead(RTC_ADDR, (uint32_t *)&rtcData, sizeof(rtcData)))
  {
    Serial.print("Loaded wakeup counter value from RTC: ");
    Serial.println(rtcData.wakeupCounter);
  }
  else
  {
    // If read fails (e.g., first ever boot), assume cold boot state
    rtcData.wakeupCounter = 0;
    Serial.println("RTC memory read failed or memory corrupted. Assuming Cold Boot (0).");
  }

  // Configure Pins
  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  // Configure the button pin with INPUT_PULLUP (reads LOW when pressed).
  pinMode(REBOOT_BUTTON_PIN, INPUT_PULLUP);

  if (rtcData.wakeupCounter == 0)
  {
    // --- STATE 1: Cold Boot / Initial Start ---
    Serial.println("System starting fresh (Cold Boot). Configuring Deep Sleep mode.");
    digitalWrite(LED_PIN, LOW);

    // Set the counter to 1 in memory so the next boot is recognized as a wake-up event
    rtcData.wakeupCounter = 1;
    ESP.rtcUserMemoryWrite(RTC_ADDR, (uint32_t *)&rtcData, sizeof(rtcData));

    // Enable deep sleep. The PIR signal (HIGH) on D0/GPIO16 will trigger the RST pin
    // to perform a wake-up restart. We set sleep time to 0 to sleep indefinitely.
    Serial.println("ESP8266 is now entering deep sleep (indefinite)...");
    ESP.deepSleep(0);
  }
  else
  {
    // --- STATE 2: Wakeup by PIR Sensor (rtcData.wakeupCounter == 1) ---
    Serial.println(">>> Motion detected! Waking up to permanent alert state.");

    // Permanently turn the LED ON
    digitalWrite(LED_PIN, HIGH);

    // The device falls through to the loop() function to listen for the reboot button.
    Serial.println("Alert LED (D4/GPIO2) is ON. System is actively monitoring reboot button (D5/GPIO14)...");
  }
}

void loop()
{
  // This loop ONLY runs if the device was woken up by the PIR sensor (State 2).

  // 1. Reboot Button Logic
  // Check if the button is pressed (LOW, due to INPUT_PULLUP).
  if (digitalRead(REBOOT_BUTTON_PIN) == LOW)
  {
    Serial.println("\n--- REBOOT BUTTON PRESSED ---");
    Serial.println("Initiating system restart to return to Deep Sleep standby...");

    // Debounce and allow time for the message to print
    delay(500);

    // Reset the counter to 0 in RTC memory so the device starts from State 1 (Deep Sleep) next time
    rtcData.wakeupCounter = 0;
    ESP.rtcUserMemoryWrite(RTC_ADDR, (uint32_t *)&rtcData, sizeof(rtcData));

    // Perform a software reset
    ESP.restart();
  }

  // Small delay to prevent the loop from running too fast
  delay(50);
}