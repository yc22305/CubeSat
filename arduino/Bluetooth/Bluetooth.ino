#include <Wire.h>  
char message[100]; // to record the message received from Android
int message_size; 

void setup() {
  Serial.begin(9600); //Arduino baud rate is 9600 
  Serial1.begin(9600); //ZS-040 baud rate is 9600
}

void loop() { 
  message_size = Serial1.available();
  if (message_size > 0) { 
     Serial.println("link!");
     for (int i=0; i<message_size; i++) {
         Serial.println("for loop");
         message[i] = Serial1.read(); // import one character once 
         Serial.print(message[i]);
         if (i == message_size-1)
             Serial.println(); // the message is over  
       }
   }
}
