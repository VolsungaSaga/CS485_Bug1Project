#include <stdlib.h>

#include <FlockBotMotion.h>
#include <FlockBotLCD.h>
#include <FlockBotBasics.h>

#define DIGILEN 10

char buf[DIGILEN];
char state;
int clearCounter;
int16_t irWindow[5];

int cmpfunc (const void * a, const void * b)
{
   return ( *(int*)a - *(int*)b );
}

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  initializeFlockBot();
  initializeMotion();
  initializeLCD();
  clearLCD();
  state = 0;
}

void loop() {
  robotStateToSerial();
  ++clearCounter;
  //int8_t batVolt;
  //batVolt = getBattery();
  //printLCD(batVolt, 3, 100);
  char buf[DIGILEN];
  char buf1[DIGILEN];
  char buf2[DIGILEN];
  
  int16_t dis, dis2, dis3;
  dis = getIR(pinout.irCenter);
  dis2 = getIR(pinout.irLeft);
  dis3 = getIR(pinout.irRight);
  
  unsigned counter;
  counter = 0;
  while (counter < 5)
  {
    irWindow[counter] = getIR(pinout.irCenter);
  }
  qsort(irWindow, 5, sizeof(int16_t), cmpfunc);
  
  sprintf(buf, "%d", dis);
  sprintf(buf1, "%d", dis2);
  sprintf(buf2, "%d", dis3);
      
  // put your main code here, to run repeatedly:
  switch (state) {
    // move to goal case
    case 0:
      if (!isDone() && (dis > 12000))
      {
        printLCD("Moving Forward", 0, 0);
        //Serial.println("Moving forward");
        state = 1;
      }
      else if (!isDone() && dis < 1200)
      {
        printLCD("Not moving forward", 6, 0);
        //state = 3;
      }
      
      //doAchievePoint(150, 150, GOOD_ROTATE, GOOD_FORWARD);
     printLCD(buf, 3, 10);
     printLCD(buf1, 4, 10);
     printLCD(buf2, 5, 10);
    break;
    case 1:
      doForwardToDistance(200, 500);
      state = 0;
    break;

    // follow case
    case 2:

    break;
    
    // end case
    case 3:
    if (isDone()) {
      doIdle();
      state++;
    }
    break;
  }

  if (!(clearCounter % 1000))
  {
    clearLCD();
  } 
}
