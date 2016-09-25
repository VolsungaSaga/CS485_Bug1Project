#include <stdlib.h>
#include <unistd.h>

#include <FlockBotMotion.h>
#include <FlockBotLCD.h>
#include <FlockBotBasics.h>

#define DIGILEN 10

#define X_GOAL 1220
#define Y_GOAL -920

//Sensor constants, based on what we've seen to be 'good' values for stopping

#define RIGHT_CLEARANCE 500
#define FRONT_CLEARANCE 500

char buf[DIGILEN];
char state;
int clearCounter;
int16_t irCenterWindow[5];
int16_t irRightWindow[5];
int16_t irLeftWindow[5];
char hasCalledDoAchPoint;
char hasCalledRotate;

//WALL FOLLOWING
char WallFollowState;

//FLAGS, for various things.

boolean amICorrectingLeft;
boolean amICorrectingRight;
boolean amIGoingForward;


int cmpfunc (const void * a, const void * b)
{
  return ( *(int*)a - * (int*)b );
}

int goToGoal()
{


}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initializeFlockBot();
  initializeMotion();
  initializeLCD();
  clearLCD();
  //Init state variables.
  state = 0;
  WallFollowState = 0;
  hasCalledDoAchPoint = 0;
  hasCalledRotate = 0;
  //Flag init
  amICorrectingLeft = false;
  amICorrectingRight = false;
  amIGoingForward = false;

  //Delay for 7 seconds, to allow for setting robot down and such.
  delay(3000);
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

  //int16_t dis, dis2, dis3;
  // dis = getIR(pinout.irCenter);
  //  dis2 = getIR(pinout.irLeft);
  //  dis3 = getIR(pinout.irRight);


  //Here's a rolling median filter that we're using to filter out some of the sensor noise. Every five readings,
  // we take the middle value. Should be alright.
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
      if ((irCenterWindow[2] > FRONT_CLEARANCE))
      {
        Serial.println("Going to state 1");
        Serial.flush();
        state = 1;
      }
      else
      {
        doIdle();
        Serial.println("There is a stupid object in my way");
        Serial.flush();
        state = 2;
      }
      printLCD(buf, 3, 10);
      printLCD(buf1, 4, 10);
      printLCD(buf2, 5, 10);
     //delay(3000);
      break;

    case 1:
      Serial.println("Moving forward in case 1");
      Serial.flush();
      if (!hasCalledDoAchPoint)
      {
        Serial.println("Starting aching point");
        doAchievePoint(X_GOAL, Y_GOAL, GOOD_ROTATE, GOOD_FORWARD);
        hasCalledDoAchPoint = 1;
      }
      state = 0;
      //delay(3000);
      break;

    // Rotate 90 case
    case 2:
      Serial.println("Following a wall in Case 2!");
      Serial.println("IR Sensor is ");
      Serial.println(irRightWindow[2]);
      Serial.flush();
      switch (WallFollowState) {
        case 0:
          if (!hasCalledRotate)
          {
            Serial.println("calling rotate");
            Serial.flush();
            doRotate(GOOD_ROTATE);
            hasCalledRotate = 1;
          }
          ++WallFollowState;
          break;


        case 1: //After seeing a wall, choose what action to take.
          Serial.println("Saw a wall choosing action");
          Serial.flush();
          if (irCenterWindow[2] < FRONT_CLEARANCE) { //See something to my front?
            WallFollowState = 1;
            hasCalledRotate = 1;
            Serial.println("Saw something to my front");
            Serial.flush();

          }

          else if ((irCenterWindow[2] >= FRONT_CLEARANCE) && (irRightWindow[2] < RIGHT_CLEARANCE)) { //If I don't see anything in front, but see something to my right...
            doIdle();
            WallFollowState = 2;
            Serial.println("Nothing to front something to right,");

          }
          break;

        case 2: //When I'm following the wall, check if I need to correct course.

           /* if ((irCenterWindow[2] < FRONT_CLEARANCE))
            {
              Serial.println("*************** Broke if statement");
              doIdle();
              WallFollowState = 0;
              hasCalledRotate = 0;
              break;
            }*/
        
          if (irRightWindow[2] < 180) { //If I'm too close to the wall...
            if (amICorrectingLeft == false) {
              Serial.println("Too close, correcting left");
              doCurve(0.1, GOOD_FORWARD);
              //doForward(GOOD_FORWARD);
              amICorrectingLeft = true;
              amICorrectingRight = false;
            }
            Serial.println("Too close, correcting left on outside");
            WallFollowState = 2;
          }
          else if((180 <= irRightWindow[2]) && (irRightWindow[2] <= 240)){
            if(amIGoingForward == false){
              doForward(GOOD_FORWARD);
              Serial.println("im in the right spot omg");
              amICorrectingLeft = false;
              amICorrectingRight = false;
              amIGoingForward = true;
            }
            WallFollowState = 2;
            Serial.println("im in the right spot on the outside");
          }  
          else if ((240 < irRightWindow[2]) && (irRightWindow[2] <= 500)) { //Am I slowly veering away from the wall?
            if (amICorrectingRight == false) {
              Serial.println("Within the strip, correcting right");
              doCurve(-1, GOOD_FORWARD);
              //doForward(GOOD_FORWARD);
              amICorrectingLeft = false;
              amICorrectingRight = true;

            }
            WallFollowState = 2;
          }

          else { //Have I suddenly lost visual with the wall?
            doIdle();
            amICorrectingLeft = false;
            amICorrectingRight = false;
            Serial.println("Called idle and stopped because I am an idiot@@@@@@");
            WallFollowState = 3;

          }
          break;
      }//end inner switch   
      hasCalledDoAchPoint = 0;
      //delay(3000);
      break;  
      // end case in the outer switch
    case 3:
      //delay(3000);
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
