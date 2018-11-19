///This code is to test the serial connection of the raspberry pi to the arduino
//using the wiringpi's built in serial library


#include <stdio.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <string.h>
#include <errno.h>
#include <curses.h>


char v;
int main(void){
	do {
	v = getch();
	} while (!isalpha(v));
//	char v = getch();
	printf("The char entered is %d",v);
	return 0;
}


