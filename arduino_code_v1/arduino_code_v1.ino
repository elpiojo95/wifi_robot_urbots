#include <SoftwareSerial.h>
#include <DRV8835MotorShield.h>

SoftwareSerial mySerial(0, 1); // RX, TX on Arduino
DRV8835MotorShield motors;

void setup() {
  Serial.begin(9600);
  mySerial.begin(115200);
}
void loop() {
  int left_motor;
  int right_motor;
  byte byte_aux1=0;
  byte byte_aux2=0;
  unsigned int int_aux1=0;
  unsigned int int_aux2=0;
  if (mySerial.available()) {
    left_motor = 0;
    byte_aux1 = 0;
    int_aux1 = 0;
    // Priemer int //
    left_motor = mySerial.read();
    byte_aux1 = mySerial.read();
    int_aux1 = byte_aux1 << 8;
    left_motor = left_motor | int_aux1;
    // Segundo int //
    right_motor = mySerial.read();
    byte_aux2 = mySerial.read();
    int_aux2 = byte_aux2 << 8;
    right_motor = right_motor | int_aux2;

    
    // DEBUG //

    Serial.print("\n//////////");
    Serial.print("\nValor int 1 recibido: ");
    Serial.println(left_motor);
    Serial.print("\nValor int 2 recibido: ");
    Serial.println(right_motor);
    if(abs(left_motor) < 400 && abs(right_motor) < 400){
      motors.setM1Speed(left_motor);
      motors.setM2Speed(right_motor);
    }
  }
}
