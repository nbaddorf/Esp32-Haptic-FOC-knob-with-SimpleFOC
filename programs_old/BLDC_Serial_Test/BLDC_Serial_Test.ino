#include <analogWrite.h>

int num;

int m1 = 13;
int m2 = 12;
int m3 = 14;
int en = 27;

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
pinMode(m1, OUTPUT);
pinMode(m2, OUTPUT);
pinMode(m3, OUTPUT);
pinMode(en, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
 //numtest = Serial1.parseInt();
   num = Serial.parseInt();
 
   if (Serial.read() == '\n') {
   if (num == 1) {
    analogWrite(m1, 100);
   } else if (num == 2) {
    analogWrite(m1, 0);
   } else if (num == 3) {
    analogWrite(m2, 100);
   } else if (num == 4) {
    analogWrite(m2, 0);
   } else if (num == 5) {
    analogWrite(m3, 100);
   } else if (num == 6) {
    analogWrite(m3, 0);
   } else if (num == 7) {
    analogWrite(en, 100);
   } else if (num == 8) {
    analogWrite(en, 0);
   }
  
  }
 
 }
}
