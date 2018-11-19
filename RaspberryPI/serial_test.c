//This code is to test the serial connection of the raspberry pi to the arduino
//using the wiringpi's built in serial library


#include <stdio.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <string.h>
#include <errno.h>
#include <curses.h>



int main(void){


int  baud_rate  = 9600;
char *device = "/dev/ttyACM1";
int handle;
int count;
unsigned int nextTime;
char value;
char v;
int i;
if((handle = serialOpen(device, baud_rate)) < 0){
	fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
	return 1;
}

if (wiringPiSetup() == -1){
	fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno));
	return 1;
}

initscr();
nodelay(stdscr,0);

while(1) {

//  while((value = getchar()) != EOF){}

while(!isalpha(value = getch())) {}

  fflush(stdout);
  serialPutchar(handle, value);

  printf("the value is %d", value);

}
}

