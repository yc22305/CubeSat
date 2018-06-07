#include <Wire.h> 

#define MSG_BUf_LENGTH 64

char message[MSG_BUf_LENGTH]; // to record the message received from Android
int msg_size;
int msg_get;

void setup() {
  Serial.begin(9600); //Arduino baud rate is 9600 
  Serial1.begin(9600); //ZS-040 baud rate is 9600
}

void loop() { 
  msg_get = Serial1.available(); // to check if there is data
  if (msg_get > 0) {
     delay(50); // leave some time to write data 
     msg_size = Serial1.available(); // the size of the data
     for (int i=0; i<MSG_BUf_LENGTH; i++) { // initialization
         message[i] = 0; 
        }
     Serial.println("link!");
     for (int i=0; i<msg_size; i++) {
         message[i] = Serial1.read(); // import one character once 
         if (i == msg_size-1)
             Serial.println(message); // the message is over  
       }
   }
}
