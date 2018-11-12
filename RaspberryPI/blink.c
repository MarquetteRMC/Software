#include <stdio.h>
#include <wiringPi.h>
#include <stdbool.h>

int main (void)
{
  wiringPiSetup () ;
  pinMode (0, OUTPUT) ;
  pinMode (3, INPUT);

for (;;)
  {
    digitalWrite (0, HIGH) ; //delay (500) ;
   // digitalWrite (0,  LOW) ; delay (500) ;
   if (digitalRead(3) == HIGH){
	digitalWrite(0, LOW);
	}
   delay (500);

  }

  return 0 ;
}
