#include <AccelStepper.h>  // by Mike McCauley https://github.com/waspinator/AccelStepper
#include <MultiStepper.h>  // inside AccelStepper, more info: https://www.airspayce.com/mikem/arduino/AccelStepper/classMultiStepper.html#details
#include <EEPROM.h>

long goBackSteps = 200;     // number of steps to go back after an endstop is encountered
int defaultLaserPos = -1400;
int laserPosTmp = -defaultLaserPos;   // to be printed positive, so it needs a -
int laserPos = -defaultLaserPos;
int gaugeAngle = 90;
int endStop = 0;
int latestLaserPos = 0;

// Variables for serial communication 
byte msg[7]; // read message coming from serial port (7 bytes for touch events)
// Example:
// 0x65 touch event
// 0x00 page
// 0x03 component ID
// 0x00 release 
// 0xFF 0xFF 0FF end of message
#define MAXRANGE 99999 

// initialization of variables correspondig to elements of Nextion screen
int p1n0 = 0;
int p1n1 = 0;
int p1n2 = 1;
int p1n3 = 0;
// temporary values for the digits in page 1
int p1n0tmp = 0;
int p1n1tmp = 0;
int p1n2tmp = 1;
int p1n3tmp = 0;

int p2n0 = 0;
int p2n1 = 0;
int p2n2 = 0;
int p2n3 = 0;
// temporary values for the digits in page 2
int p2n0tmp = 0;
int p2n1tmp = 0;
int p2n2tmp = 0;
int p2n3tmp = 0;

int *pDigit = nullptr; // pointer to value of digit
const char * nexCtrl = nullptr; // name of nextion UI control
const char * digits[4] = {"n0", "n1", "n2", "n3"};
int *p1DigitVal[4] = {&p1n0, &p1n1, &p1n2, &p1n3};
int *p1DigitValtmp[4] = {&p1n0tmp, &p1n1tmp, &p1n2tmp, &p1n3tmp};
int *p2DigitVal[4] = {&p2n0, &p2n1, &p2n2, &p2n3};
int *p2DigitValtmp[4] = {&p2n0tmp, &p2n1tmp, &p2n2tmp, &p2n3tmp};

const int enPin1 = 8;
const int dirPin1 = 5;  //pin for the direction, aka positive or negative sign of the movement command
const int stepPin1 = 2;
const int enPin2 = 8;
const int dirPin2 = 6;
const int stepPin2 = 3;


const int switchUpPin = A0;
const int switchDownPin = A1;
const int switchHomePin = A2;
const int initFindZeroPin = A3;


const int endStopPin1 = 9;     //->LOW
const int endStopPin2 = 10;    //->HIGH
const int triggerOutPin = 12;


const int stepFraction = 64;  //working @1/64 step fraction (jumper in the middle)
const int stepManual = 1;

long stepTrigOut = 0;  //if !=0, turn pinMode OUTPUT, else, INPUT-> no risk of damages. number of step for generating triggerOut Signal [0,5V]
long startPos1 = 0;
long startPos2 = 0;

long stepFinalPos = 0;

int tau2m1 = 20;
int tau2m2 = 20;

bool flagAknStep1 = false;        //true-> display, false->not display
bool flagAknManual = true;        //true-> manual input-> not display; false->serial input->display Acknowledgement
bool serialDisplayEnable = true;  //true ->display enabled, false->display disabled


String inputStringPC;
String inputStringNextion;  //from nextion to ard
String sendString;          //from ard to nextion

AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1);  //(DRIVER, STEP, DIR)
MultiStepper steppers;


// pointers initialized to stepper 1 
const int *pDirPin = &dirPin1;
const int *pStepPin = &stepPin1;
const int *pEnPin = &enPin1;
int *pTau2m = &tau2m1;
AccelStepper *pStepper = &stepper1;

int numMot = 1;
char enable = 'r';
int stepRel = 100;       // MAX 2147483647 STEPS
int step = 0;

int foundEndstop1 = 0;
int foundEndstop2 = 0;
char cmd[32];

enum Movement {
  CW, // clockwise
  CCW // counterclockwise
};
Movement oldMovement = CW;
Movement newMovement = CW;

int sign = 1;
int backlashCompensation = 100;



// function prototypes
void ShowMsg(const char * message);
void ShowNum(int n);
int GoToEndstop(AccelStepper stepper, int endStop);  //==1, zero found; ==0 zero not found;
void IncreaseDigit(int* pDigit, const char * nexCtrl );
void DecreaseDigit(int* pDigit, const char * nexCtrl );
void ExecuteString(int numMot, char enable, int step);
void CompensateBacklash(Movement* pOld, Movement* pNew);


/*******************************************************/

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);

  /* setup Gaia*/
  Serial.println("setTimeout");
  Serial.setTimeout(1000);      //10

  //stepper settings
  Serial.println("settings of stepper motors");
  stepper1.setMaxSpeed(2000);  //5000
  stepper1.setSpeed(1000);  //5000
  stepper1.setAcceleration(300000);
  steppers.addStepper(stepper1);

  //pinMode
  pinMode(endStopPin1, INPUT);
  pinMode(endStopPin2, INPUT);
  pinMode(switchUpPin, INPUT_PULLUP);
  pinMode(switchDownPin, INPUT_PULLUP);
  pinMode(switchHomePin, INPUT_PULLUP);
  //pinMode(displayEnable, INPUT_PULLUP);
  pinMode(triggerOutPin, INPUT_PULLUP);
  pinMode(initFindZeroPin, LOW);
  pinMode(enPin1, OUTPUT);
  
  //read latest laser position and update the num box at page 3
  EEPROM.get(0,latestLaserPos); // address 0 is where the latest laser position is stored
  Serial1.print("p[3].n2.val=");
  Serial1.print(latestLaserPos);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  laserPos = -latestLaserPos;


  // initZero: procedura di setup per trovare lo zero all'accensione
  ShowMsg("InitZero");
  if (digitalRead(initFindZeroPin) == LOW) {
    digitalWrite(enPin1, LOW);
    if (endStopPin1 == HIGH) {
      stepper1.runToNewPosition(-500);
    } else if (endStopPin2 == HIGH) {
      stepper1.runToNewPosition(500);
    } 
    
    // find the endstops
    foundEndstop2 = GoToEndstop(stepper1, 2);
    foundEndstop1 = GoToEndstop(stepper1, 1);
    
    if (foundEndstop1 == 0 || foundEndstop2 == 0) {
      ShowMsg("ErrorInitZero");
    }

    delay(300);
    // Serial.println("initZero");
    // Serial.println(" ");
    
    pinMode(initFindZeroPin, INPUT_PULLUP);   // cambio; "ora hai trovato lo zero iniziale"
  } else {
    digitalWrite(enPin1, HIGH);
  }  //disable driver

  // display default laser position on page 3
  Serial1.print("p[3].n1.val=");      // "default laser position"
  Serial1.print(-defaultLaserPos);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  ShowMsg("OKinitZero");
}

void loop() {   // put your main code here, to run repeatedly:
  if (Serial1.available() >=  7){ // check repeatedly Serial
    for (int k = 0; k<7; k++){
      msg[k] = Serial1.read();
    }
  

    switch(msg[1]) // check page
    {
    /*************************PAGE 0************************************/
      case 0: // page 0
        switch(msg[2]) // components
        {
          case 3: // Set Step button 
          { 
            // set the values of the digits to corresponding values
            for(int i = 0; i<4; i++){
              Serial1.print("p[1].");
              Serial1.print(digits[i]);
              Serial1.print(".val=");
              Serial1.print(*p1DigitVal[i]);
              Serial1.write(0xFF);
              Serial1.write(0xFF);
              Serial1.write(0xFF);
            }

            // send stepRel variable to the "Step set to:" numeric text box on page 1 
            Serial1.print("p[1].n4.val=");
            Serial1.print(stepRel);
            Serial1.write(0xFF);
            Serial1.write(0xFF);
            Serial1.write(0xFF);
    
          }
            break;
          case 4: // arrow up
            enable = 'r'; 
            newMovement = CCW;
            CompensateBacklash(&oldMovement, &newMovement);
            ExecuteString(numMot, enable, stepRel);
            break;
          case 5: // arrow down
            enable = 'r';
            newMovement = CW;
            CompensateBacklash(&oldMovement, &newMovement); 
            ExecuteString(numMot, enable, -stepRel);
            break;
          case 6:   // "set laser pos" button
            laserPosTmp = int(- pStepper->currentPosition());
            Serial1.print("p[3].n0.val=");      // "new laser position"
            Serial1.print(laserPosTmp);
            Serial1.write(0xff);
            Serial1.write(0xff);
            Serial1.write(0xff);
            break;
          case 7: // Go To button 
          { 
            // set the values of the digits to corresponding values
            for(int i = 0; i<4; i++){
              Serial1.print("p[2].");
              Serial1.print(digits[i]);
              Serial1.print(".val=");
              Serial1.print(*p2DigitVal[i]);
              Serial1.write(0xFF);
              Serial1.write(0xFF);
              Serial1.write(0xFF);
            }    
          }
            break;
        }
        break; // end page 0
  /*************************PAGE 1************************************/
      case 1: 
        switch(msg[2])
        {
          case 9: // arrow up n3
            pDigit = &p1n3tmp;
            nexCtrl = "n3";
            IncreaseDigit(pDigit, nexCtrl);
            break;
          case 10: // arrow down 3
            pDigit = &p1n3tmp;
            nexCtrl = "n3";
            DecreaseDigit(pDigit, nexCtrl );
            break;
          case 8: // arrow up 2
            pDigit = &p1n2tmp;
            nexCtrl = "n2";
            IncreaseDigit(pDigit, nexCtrl );
            break;
          case 11: // arrow down 2
            pDigit = &p1n2tmp;
            nexCtrl = "n2";
            DecreaseDigit(pDigit, nexCtrl );
            break;   
          case 7: // arrow up 1
            pDigit = &p1n1tmp;
            nexCtrl = "n1";
            IncreaseDigit(pDigit, nexCtrl );
            break;
          case 12: // arrow down 1
            pDigit = &p1n1tmp;
            nexCtrl = "n1";
            DecreaseDigit(pDigit, nexCtrl );
            break;  
          case 6: // arrow up 0
            pDigit = &p1n0tmp;
            nexCtrl = "n0";
            IncreaseDigit(pDigit, nexCtrl );
            break;
          case 13: // arrow down 0
            pDigit = &p1n0tmp;
            nexCtrl = "n0";
            DecreaseDigit(pDigit, nexCtrl );
            break;  
          case 14: // Reset button
            { 
              p1n0tmp = 0;
              p1n1tmp = 0;
              p1n2tmp = 1;
              p1n3tmp = 0;
              
              for (int i=0; i <4; i++){
                Serial1.print(digits[i]);
                Serial1.print(".val=");
                Serial1.print(*p1DigitValtmp[i]);
                Serial1.write(0xFF);
                Serial1.write(0xFF);
                Serial1.write(0xFF);
              }
            }
            break;
          case 15: // ok
          {
            // update true values of digits
            for(int i = 0; i<4; i++){
              *p1DigitVal[i] = *p1DigitValtmp[i];
            }

            stepRel = (p1n3 * 1000) + (p1n2 * 100) + (p1n1 * 10) + (p1n0);
            Serial1.print("p[1].n4.val=");
            Serial1.print(stepRel);
            Serial1.write(0xFF);
            Serial1.write(0xFF);
            Serial1.write(0xFF);

            Serial1.print("p[0].n0.val=");
            Serial1.print(stepRel);
            Serial1.write(0xFF);
            Serial1.write(0xFF);
            Serial1.write(0xFF);
          }
            break;
          case 1: // back 
            {
              for(int i=0; i<4; i++){
                *p1DigitValtmp[i]=*p1DigitVal[i];
              }          
            }
            break;
        }
        break; // end page 1
  /*************************PAGE 2************************************/
      case 2:
        switch(msg[2])
        {
          case 9: // arrow up n3
            pDigit = &p2n3tmp;
            nexCtrl = "n3";
            IncreaseDigit(pDigit, nexCtrl );
            break;
          case 10: // arrow down 3
            pDigit = &p2n3tmp;
            nexCtrl = "n3";
            DecreaseDigit(pDigit, nexCtrl );
            break;
          case 8: // arrow up 2
            pDigit = &p2n2tmp;
            nexCtrl = "n2";
            IncreaseDigit(pDigit, nexCtrl );
            break;
          case 11: // arrow down 2
            pDigit = &p2n2tmp;
            nexCtrl = "n2";
            DecreaseDigit(pDigit, nexCtrl );
            break;   
          case 7: // arrow up 1
            pDigit = &p2n1tmp;
            nexCtrl = "n1";
            IncreaseDigit(pDigit, nexCtrl );
            break;
          case 12: // arrow down 1
            pDigit = &p2n1tmp;
            nexCtrl = "n1";
            DecreaseDigit(pDigit, nexCtrl );
            break;  
          case 6: // arrow up 0
            pDigit = &p2n0tmp;
            nexCtrl = "n0";
            IncreaseDigit(pDigit, nexCtrl );
            break;
          case 13: // arrow down 0
            pDigit = &p2n0tmp;
            nexCtrl = "n0";
            DecreaseDigit(pDigit, nexCtrl );
            break;
          case 14:    // lamp button
            enable = 'a';
            step = 0;
            ExecuteString(numMot, enable, step);
            break;
          case 16:    // laser button
            enable = 'a';
            step = laserPos; // already negative (for motor sign convention)
            ExecuteString(numMot, enable, step);
            break;
          case 15: // OK
            // update true values of digits
            for(int i = 0; i<4; i++){
              *p2DigitVal[i] = *p2DigitValtmp[i];
            }
            enable = 'a';
            step = (p2n3 * 1000) + (p2n2 * 100) + (p2n1 * 10) + (p2n0); 
            ExecuteString(numMot, enable, -step);

            break;
          case 1:// back
            for(int i=0; i<4; i++){
              *p2DigitValtmp[i]=*p2DigitVal[i];
            }
            break;

          case 17:// reset
          { 
            p2n0tmp = 0;
            p2n1tmp = 0;
            p2n2tmp = 0;
            p2n3tmp = 0;
            
            for (int i=0; i <4; i++){
              Serial1.print(digits[i]);
              Serial1.print(".val=");
              Serial1.print(*p2DigitValtmp[i]);
              Serial1.write(0xFF);
              Serial1.write(0xFF);
              Serial1.write(0xFF);
            }
          }

            break;

        }
        break; // end page 2

  /*************************PAGE 3************************************/
      case 3:

        switch(msg[2])
        {
        case 1:   // "yes" = overwrite
          // latest
          latestLaserPos = int(-pStepper->currentPosition());

          // update the value in the EEPROM
          EEPROM.put(0,latestLaserPos);

          // update the displayed value
          Serial1.print("p[3].n2.val=");
          Serial1.print(latestLaserPos);
          Serial1.write(0xFF);
          Serial1.write(0xFF);
          Serial1.write(0xFF);

          //call the ExecuteString function with appropriate parameters
          enable = 'p';
          ExecuteString(numMot, enable, 0);
          break;
        case 3:   // "no" = back

          break;
        } 
        break;  // end page 3

    } // end switch page
  } // end if Serial1.availble
} // end void loop


// function definitions
void IncreaseDigit(int* pDigit, const char * nexCtrl ){
  if(*pDigit == 9) *pDigit = 0;
  else (*pDigit)++;
  Serial1.print(nexCtrl);
  Serial1.print(".val=");
  Serial1.print(*pDigit);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
}

void DecreaseDigit(int* pDigit, const char * nexCtrl ){
  if(*pDigit == 0) *pDigit = 9;
  else (*pDigit)--;
  Serial1.print(nexCtrl);
  Serial1.print(".val=");
  Serial1.print(*pDigit);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
}

void ShowMsg(const char * message){
  Serial1.print("p[0].t1.txt=\"");
  Serial1.print(message);
  Serial1.print("\"");
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
  delay(500);
}

void ShowNum(int n){
  Serial1.print("p[0].n1.val=");
  Serial1.print(n);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
}


void ExecuteString(int numMot, char enable, int step){
  // print corresponding command on Nextion 
  snprintf(cmd, sizeof(cmd), "%d%c%d", numMot, enable, step);

  // print on Nextion
  Serial1.print("p[0].t1.txt=\"");
  Serial1.print(cmd);
  Serial1.print("\"");
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);

  switch (enable) {  //sorting serial functions
    case 'a':  //absolute movement
      digitalWrite(*pEnPin, LOW);
      // every absolute movement goes back to endstop 1 first, which sets oldMovement = CW
      if (GoToEndstop(*pStepper, 1) == 1) {
        pStepper->setCurrentPosition(0);
        pStepper->moveTo(step);
      }
      break;
    case 'r':  //relative movement
      digitalWrite(*pEnPin, LOW);
      pStepper->move(step);      
      break;
    case 'p':  // save current position as default "laser" position // WHY NOT WRITE IT DIRECTLY IN THE BUTTON RELEASE EVENT IN PAGE 3 RATHER THAN CREATE A PARTICLUAR CASE????
       laserPos = pStepper->currentPosition();
      break;
  } // end of switch (enable)
  

  //acknowledgements
  flagAknManual = true;
  if (digitalRead(switchUpPin) == LOW) {
    digitalWrite(*pEnPin, LOW);
    pStepper->move(stepManual);
    flagAknManual = true;
  }
  if (digitalRead(switchDownPin) == LOW) {
    digitalWrite(*pEnPin, LOW);
    pStepper->move(-stepManual);
    flagAknManual = true;
  }
  if (digitalRead(switchHomePin) == LOW) {
    digitalWrite(*pEnPin, LOW);
    pStepper->moveTo(0);
    flagAknManual = true;
  }


  ShowNum(int(- pStepper->currentPosition()));
  while (pStepper->distanceToGo()!=0) {    // movement of stepper motor
    if (pStepper->currentPosition() % 50 == 0) {
      ShowNum(int(- pStepper->currentPosition()));
      //no update of gauge display during movement, just start and final position
    }
    // EndStops: check if an endstop is encountered
    if (pStepper->isRunning()==true) {
      if (digitalRead(endStopPin1) == HIGH) {     // Endstop1: minimum angle for mirror holder
        digitalWrite(*pEnPin, LOW);  //enable motor driver
        pStepper->stop();
        // Serial1.println("ENDSTOP 1");
        pStepper->runToNewPosition(pStepper->currentPosition() - goBackSteps);    // - bc of sign convention; goes clockwise
        pStepper->setCurrentPosition(0);
        oldMovement = CW;
        //delay(100);
        break;
      }
      if (digitalRead(endStopPin2) == HIGH) {       // Endstop2: max angle for mirror holder
        digitalWrite(*pEnPin, LOW);  //enable motor driver
        pStepper->stop();
        // Serial1.println("ENDSTOP 2");
        pStepper->runToNewPosition(pStepper->currentPosition() + goBackSteps);
        oldMovement = CCW;
        break;
        // at this point, we do NOT know the position with absolute confidence, due to non-null backlash between the gears! 
      }
    }
    pStepper->run();
  }   // end of while
  ShowNum(int(- pStepper->currentPosition()));

  // gauge shows position
  gaugeAngle = int(map(int(- pStepper->currentPosition()), 0, 12800, 90, 450));
  Serial1.print("p[0].z0.val=");      // refresh or bug fix needed!!
  Serial1.print(gaugeAngle);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

} //end of function ExecuteString


int GoToEndstop(AccelStepper stepper, int endStop) {
  long d2go = 0;
  digitalWrite(enPin1, LOW);                                            //enable motor driver
  if (endStop == 1){
    stepper.move(MAXRANGE);                                             //set movement target and distanceToGo attribute
  }else if(endStop == 2){
    stepper.move(-MAXRANGE);
  }
  while (stepper.distanceToGo() != 0) {           // while no endstop encountered and maxSteps not reached
    if ((digitalRead(endStopPin1) == HIGH )&& (endStop==1)) {  // endstop 1 encountered
      stepper.stop();
      d2go = stepper.distanceToGo();
      stepper.runToNewPosition(stepper.currentPosition() - goBackSteps);
      oldMovement = CW;
      break;
    }
    if ((digitalRead(endStopPin2) == HIGH )&& (endStop==2)) {     // endstop 2 encountered
      stepper.stop();
      d2go = stepper.distanceToGo();
      stepper.runToNewPosition(stepper.currentPosition() + goBackSteps);
      oldMovement = CCW;
      break;
    }
    stepper.run();  //run movement
    //no update of gauge display during movement, just start and final position
  }
  
  if (d2go != 0) {
    return 1;  // endostop 1 or 2 found
  } else {
    return 0;  // Error - endstop not found
  }
}

void CompensateBacklash(Movement* pOld, Movement* pNew)
{
  if(*pNew!=*pOld){
    if(*pNew == CW && *pOld == CCW){
      sign = -1; 
      ShowMsg("CW Compensation"); // debug
      delay(2000);
    } else if (*pNew == CCW && *pOld == CW){
      sign = +1;
      ShowMsg("CCW Compensation"); // debug
      delay(2000);
    }

    long positionTmp = pStepper->currentPosition();
    pStepper->runToNewPosition(pStepper->currentPosition()+sign * backlashCompensation);
    pStepper->setCurrentPosition(positionTmp); // restore position prior to backlash compensation
    
    ShowMsg("Compensated"); // debug
    delay(1000);
  }
  *pOld = *pNew;


}


