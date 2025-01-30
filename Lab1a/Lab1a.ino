#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;

void setup() 
{
  // put your setup code here, to run once:

  Serial.begin(57600);
  delay(1000);
  buzzer.play("c32");
  

}

