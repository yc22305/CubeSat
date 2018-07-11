// relays
#define RELAY_ONE 13 // power control of other relays
#define RELAY_TWO 12 // negative yaw
#define RELAY_THREE 11 // main thruster
#define RELAY_FOUR 10 // poitive yaw

int i = 0;

void setup() {  
  Serial.begin(115200);
  
  pinMode(RELAY_ONE, OUTPUT);
  pinMode(RELAY_TWO, OUTPUT);
  pinMode(RELAY_THREE, OUTPUT);
  pinMode(RELAY_FOUR, OUTPUT);
  digitalWrite(RELAY_ONE, LOW);
  digitalWrite(RELAY_TWO, LOW);
  digitalWrite(RELAY_THREE, LOW);
  digitalWrite(RELAY_FOUR, LOW);
  
  delay(1000);
}

void loop() {
  Serial.println(i);
  delay(1000);
  
  // power on relays and switch on each one
  if (i==0) {
     digitalWrite(RELAY_ONE, HIGH);
     i++;
     delay(4000);
     Serial.println("power on");
     return;
    }
  if (i==1) {
     digitalWrite(RELAY_TWO, HIGH);
     i++;
     return;
    }
  if (i==2) {
     digitalWrite(RELAY_THREE, HIGH);
     i++;
     return;
    }
  if (i==3) {
     digitalWrite(RELAY_FOUR, HIGH);
     i++;
     return;
    }

  // switch off each one
  if (i==4) {
     digitalWrite(RELAY_FOUR, LOW);
     i++;
     return;
    }
  if (i==5) {
     digitalWrite(RELAY_THREE, LOW);
     i++;
     return;
    }
  if (i==6) {
     digitalWrite(RELAY_TWO, LOW);
     i++;
     return;
    }

  // power off relays and try to switch
  if (i==7) {
     digitalWrite(RELAY_ONE, LOW);
     i++;
     return;
    }
  if (i==8) {
     digitalWrite(RELAY_FOUR, HIGH);
     i++;
     return;
    }
  if (i==9) {
     digitalWrite(RELAY_THREE, HIGH);
     i++;
     return;
    }
  if (i==10) {
     digitalWrite(RELAY_TWO, HIGH);
     i++;
     return;
    }

  // reset
  if (i==11) {
     digitalWrite(RELAY_ONE, LOW);
     digitalWrite(RELAY_TWO, LOW);
     digitalWrite(RELAY_THREE, LOW);
     digitalWrite(RELAY_FOUR, LOW);
     while(1);
    }
}
