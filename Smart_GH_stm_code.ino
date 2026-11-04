// =====================================================================
//  STM32F103C8T6 - Greenhouse Slave Actuator Controller 
//  - Controls 12V PC Fan via IRLZ44N MOSFET on PA0
//  - Controls SG90 Servo Motor on PA6
//  - Listens to ESP32 Master via Serial1 (PA10 RX / PA9 TX)
// =====================================================================
#include <Servo.h>

#define FAN_PIN   PA0   // Connected via 100/1k resistor to MOSFET Gate
#define SERVO_PIN PA6   // SG90 Servo motor signal

Servo actuatorServo;

void setup() {
  // Initialize USB Serial for PC debugging (optional, if plugged in)
  Serial.begin(115200);   
  
  // Initialize UART port linked to the ESP32 (TX2/RX2)
  Serial1.begin(115200);  
  
  // Configure the Fan MOSFET pin
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW); // Ensure fan starts OFF on boot

  // Configure the Servo pin
  actuatorServo.attach(SERVO_PIN);
  actuatorServo.write(0);     // Default to closed position (0 degrees)

  Serial.println("STM32 Actuator Slave Ready.");
}

void loop() {
  // Check if data is available from the ESP32
  if (Serial1.available() > 0) {
    // Read the incoming string until a newline character is found
    String packet = Serial1.readStringUntil('\n');
    packet.trim(); // Remove whitespace or carriage returns

    // Validate that the packet has proper < and > boundaries
    if (packet.startsWith("<") && packet.endsWith(">")) {
      int startIdx = packet.indexOf('<');
      int endIdx = packet.indexOf('>');
      
      // Ensure the indices are valid before extracting the substring
      if (startIdx >= 0 && endIdx > startIdx) {
        String content = packet.substring(startIdx + 1, endIdx);

        // --- ACTUATOR CONTROL PARSING ---
        
        if (content == "FAN_ON") {
          digitalWrite(FAN_PIN, HIGH);
          Serial.println("Command Received: Fan ON");
        } 
        else if (content == "FAN_OFF") {
          digitalWrite(FAN_PIN, LOW);
          Serial.println("Command Received: Fan OFF");
        } 
        else if (content == "VENT_OPEN") {
          actuatorServo.write(90); // Rotate servo to 90 degrees (Open Vent)
          Serial.println("Command Received: Servo Vent OPEN");
        } 
        else if (content == "VENT_CLOSE") {
          actuatorServo.write(0);  // Rotate servo to 0 degrees (Close Vent)
          Serial.println("Command Received: Servo Vent CLOSED");
        }
        
        // Note: If the content is the regular sensor telemetry packet 
        // (e.g., "<45,1200.5,28.4..."), the STM32 simply ignores it here,
        // safely dropping it without executing a fan/servo command.
      }
    }
  }
}