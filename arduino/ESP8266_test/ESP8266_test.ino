#define BAUD_WIFI 9600
#define BAUD_Serial 57600


void setup() {
  Serial.begin(BAUD_Serial);
  Serial1.begin(57600); // initial baud rate for communicate with ESP8266
  delay(1000);
    Serial.println("Start...");
 // Serial.print("Setting BAUDRATE to "); Serial.println(BAUD_WIFI);
 // Serial1.print("AT+IPR="); Serial1.println(BAUD_WIFI);
 // delay(1000)
 // Serial1.begin(BAUD_WIFI);
  delay(1000); 
}
 
void loop() {
  if (Serial.available() > 0) {
     String command = Serial.readStringUntil('\n');
     Serial1.print(command);
   }

  if (Serial1.available() > 0) {
     String response = Serial1.readStringUntil('\n');
     Serial.print(response); 
    }
}


