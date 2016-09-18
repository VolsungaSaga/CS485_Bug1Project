#include <stdlib.h>
#include <unistd.h>

#include <FlockBotMotion.h>
#include <FlockBotLCD.h>
#include <FlockBotBasics.h>

#define DIGILEN 10

char buf[DIGILEN];
char state;
int clearCounter;
int16_t irCenterWindow[5];
int16_t irRightWindow[5];
int16_t irLeftWindow[5];


int cmpfunc (const void * a, const void * b)
{
   return ( *(int*)a - *(int*)b );
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initializeFlockBot();
  initializeMotion();
  initializeLCD();
  clearLCD();
  state = 0;
}

void loop() {
//  robotStateToSerial();
  ++clearCounter;
  //int8_t batVolt;
  //batVolt = getBattery();
  //printLCD(batVolt, 3, 100);
  char buf[DIGILEN];
  char buf1[DIGILEN];
  char buf2[DIGILEN];
  
  unsigned counter;
  counter = 0;
  while (counter < 5)
  {
    irCenterWindow[counter] = getIR(pinout.irCenter);
    irRightWindow[counter] = getIR(pinout.irRight);
    irLeftWindow[counter] = getIR(pinout.irLeft);
    ++counter;
  }
  qsort(irCenterWindow, 5, sizeof(int16_t), cmpfunc);
  qsort(irRightWindow, 5, sizeof(int16_t), cmpfunc);
  qsort(irLeftWindow, 5, sizeof(int16_t), cmpfunc);
  
  Serial.println(irCenterWindow[3]);
  Serial.flush();
  sprintf(buf, "%d", irCenterWindow[3]);
  sprintf(buf1, "%d", irRightWindow[3]);
  sprintf(buf2, "%d", irLeftWindow[3]);
      
  // put your main code here, to run repeatedly:
  switch (state) {
    // move to goal case
    case 0:
      if ((irWindow[3] > 500))
      {
        Serial.println("Going to state 1");
        Serial.flush();
        state = 1;
      }
      else
      {
        Serial.println("There is a stupid object in my way");
        Serial.flush();
        state = 2;
      }
     printLCD(buf, 3, 10);
     printLCD(buf1, 4, 10);
     printLCD(buf2, 5, 10);
    break;
    case 1:
      Serial.println("Moving forward in case 1");
      Serial.flush();
      doForwardToDistance(200, 100);
      state = 0;
    break;

    // Rotate 90 case
    case 2:
      
    break;
    
    // end case
    case 3:
    if (isDone()) {
      Serial.println("In case 3!");
      doIdle();
      state++;
      exit(0);
    }
    break;
  }

  if (!(clearCounter % 1000))
  {
    clearLCD();
  } 
}
