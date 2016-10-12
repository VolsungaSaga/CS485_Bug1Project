#include <FlockBotMotion.h>

#include <FlockBotLCD.h>

#include <FlockBotBasics.h>

#define DIGILEN 10

#define X_GOAL 1220

#define Y_GOAL -920
char state;



void setup() {
  // put your setup code here, to run once:
  initializeFlockBot();
  initializeMotion();
  initializeLCD();
  clearLCD();
  state = 0;
}

void loop() {
  // put your main code here, to run repeatedly:

  //Output position/sensor data to LCD, first and foremost.
  char bufferCenterIR [DIGILEN] ;
  char bufferRightIR [DIGILEN] ;
  char bufferLeftIR [DIGILEN] ;


  int16_t centerIR = getIR(pinout.irCenter);
  int16_t frontLeftIR = getIR(pinout.irFrontLeft);
  int16_t frontRightIR = getIR(pinout.irFrontRight);

  sprintf(bufferCenterIR, "%d", centerIR);
  sprintf(bufferRightIR, "%d", frontRightIR);
  sprintf(bufferLeftIR, "%d", frontLeftIR);

  printLCD(bufferCenterIR, 0, 0);
  printLCD(bufferRightIR, 1, 0);
  printLCD(bufferLeftIR, 2, 0);

  
  switch (state) {
    //Make a dash to the goal!
    case 0:
      doAchievePoint(X_GOAL,Y_GOAL, GOOD_ROTATE, GOOD_FORWARD);
      state++;
    break;
    
    case 1:
      if(isDone()){
       // doAchievePoint(-300,300, GOOD_ROTATE, GOOD_FORWARD);
        state++;
          }
    break;

   
  }
}
