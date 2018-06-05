#include <cstdlib>

#define BAUD 9600
float f1 = 0.525;
float f2 = 0.21;
float f3 = 56.22;
int i = 0;
char str[40];


void setup(){
  Serial.begin(9600);
}

void loop(){
  for (i=0; i<40; i++) {
      str[i] = 32;
     }
  snprintf(str, 40, "a %.2f %.2f %.2f ", f1, f2, f3);
  Serial.write(str);
  
  f1++;
  f2++;
  f3++;
  delay(500);
}
