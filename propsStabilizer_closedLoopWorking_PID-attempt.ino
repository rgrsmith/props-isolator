//Initial Commit.

//  0.006096mm/step, or 0.000762mm/eighth step
// to slew .2 deg:
// (tan(0.0034 radians)*.5m ) / ( 0.000762mm * 2) = 1120 quarter steps
// so should test at 1Hz with 560 input bits on the dac, and double it to get the full stroke (cant hit 1120 with 10 bit dac)
// --> 232 to 792 should be the analog read input
//DAC resolution is 1024 so this is fairly close, but camera resolution is like a third of this

// stepper spec says we should be able to slew at 2mm/s = 1312 quarter steps/s

#include <SPI.h>
#include <AccelStepper_closedLoop.h>
#include <MultiStepper_closedLoop.h>
#include <Encoder_Buffer.h>


// Define a stepper and the pins it will use
//AccelStepper stepper(AccelStepper::DRIVER, 22, 2);
//AccelStepper stepper_1(AccelStepper::DRIVER, 10, 24, 0, 0, true, 45);
AccelStepper stepper_1(AccelStepper::DRIVER, 10, 24, 0, 0, true, 45);
AccelStepper stepper_2(AccelStepper::DRIVER, 11, 29, 0, 0, true, 46);
AccelStepper stepper_3(AccelStepper::DRIVER, 12, 39, 0, 0, true, 48);
AccelStepper stepper_4(AccelStepper::DRIVER, 13, 39, 0, 0, true, 47); // note this ends up the last driver slot, but the corner encoder pins

MultiStepper steppers;
// Motor setpoints
long positions[4] = {0, 0, 0, 0};

//Makes the millis cycle for sine wave start at 0
long offset = 0;

//Variables for setpoint
double increment, setpoint = 0;

//Variables for reading serial line
size_t         count     = 0;
const size_t   MAX_CHARS = 64;
char           line[ MAX_CHARS ];

//Pin for reading setpoint from openMV
//#define openMV_setpoint 

//checking loop cycle time
unsigned long lastTime = 0; //Beware rollover... do not run for > 49 days continuously
unsigned long thisTime = 0;

int encoder1Reading = 0;

bool printOnce = true;

long softLimit [2] = {0, 17000/2};


int sensorPin = A10;

int oldPos = -100;

long startTime = micros();

void setup()
{  
  Serial.begin(9600);
  SPI.begin();
  stepper_1.init();
  stepper_2.init();
  stepper_3.init();
  stepper_4.init();
  Serial.println("starting script");

  // Initialize all stepper motor params / encoder params
  // Note this also sets required pins
  initStepperParams();

  pinMode(4, INPUT);
  
  // Home motors one at a time
  Serial.println("Homing motors");
  homeMot(stepper_1);
  homeMot(stepper_2);
  homeMot(stepper_3);
  homeMot(stepper_4);
  Serial.println("Pos is now: 0");
  setpoint = 0;

  // Add steppers to mlti stepper manager
  steppers.addStepper(stepper_1);
  steppers.addStepper(stepper_2);
  steppers.addStepper(stepper_3);
  steppers.addStepper(stepper_4);

  // Set max stepper speeds
  stepper_1.setMaxSpeed(1000);
  stepper_1.setSpeed(1000);
  stepper_2.setMaxSpeed(1000);
  stepper_2.setSpeed(1000);
  stepper_3.setMaxSpeed(1000);
  stepper_3.setSpeed(1000);
  //stepper_4.setMaxSpeed(1312);
  //stepper_4.setSpeed(1312);
  stepper_4.setMaxSpeed(1800);
  stepper_4.setSpeed(1800);

/*
  //Set motor positions to end of travel
  positions[0] = softLimit[1];
  positions[1] = positions[0];
  positions[2] = positions[0];
  positions[3] = positions[0];
  
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  printOnce = true;
  delay(500);
  */
  
  
  
    //Set motor positions to halfway through their travel
  positions[0] = (softLimit[1]-softLimit[0])/2;
  positions[1] = positions[0];
  positions[2] = positions[0];
  positions[3] = positions[0];
    steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  printOnce = true;
  delay(500);
  

/*  // TIMER 1 for interrupt frequency 10 Hz:
  cli(); // stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // set compare match register for 10 Hz increments
  OCR1A = 24999; // = 16000000 / (64 * 10) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 64 prescaler
  TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // allow interrupts
  */
 
  
  offset = millis();
}

void loop()
{
  //stepper_1.readEncoder();
  //Serial.println(stepper_1.readEncoder());
/*
    lastTime = thisTime;
    thisTime = millis();
  
    //encoder1Reading = -Encoder1.readEncoder();//Read Encoder
    encoder1Reading = stepper_1.readEncoder();
    //Serial.println(encoder1Reading);
      
    if(lineReady()) {
      //Serial.println(actuator1.getPos());
      increment = atoi( line );
      if (increment >= softLimit[0] && increment <= softLimit[1]) {
        setpoint = increment;
        stepper_1.moveTo(setpoint);
        //stepper_1.stop();
        printOnce = true;
        stepper_1.enableOutputs();
      }
      else {
        Serial.print("Cannot move to pos, soft limit is set to: ");
        Serial.print("[");
        Serial.print(softLimit[0]);
        Serial.print(" - ");
        Serial.print(softLimit[1]);
        Serial.println("]");
      }
    }

    
//
//    if(abs(encoder1Reading-stepper_1.currentPosition()) > 4 ){
//      stepper_1.
//    }
    
    boolean stepping = stepper_1.run();
    if(!stepping){
      stepper_1.disableOutputs();
      if(printOnce) {
        Serial.print("pos is now: ");
        Serial.println(setpoint);
        Serial.print("Encoder Reads: ");
        Serial.println(stepper_1.readEncoder());
        printOnce = false;
      }
    }
    else {
      //Serial.println(stepper_1.speed());
    }
*/
      //float hz = 1;
      //float modTime = 1000/hz;
      //long thisTime = millis() - offset;

     // positions[0] = (softLimit[1]-softLimit[0])/2 + 200*sin(thisTime%(int(modTime)) * (2* 3.1415 / (modTime)));
      //positions[1] = (softLimit[1]-softLimit[0])/2 + 200*sin(thisTime%(int(modTime)) * (2* 3.1415 / (modTime))+2*3.1415/4);
      //positions[2] = (softLimit[1]-softLimit[0])/2 + 200*sin(thisTime%(int(modTime)) * (2* 3.1415 / (modTime))+4*3.1415/4);
      //positions[3] = (softLimit[1]-softLimit[0])/2 + round(600*sin(thisTime%(int(modTime)) * (2.0* 3.1415 / (modTime))+4.0*3.1415/4.0));
      //Serial.println((softLimit[1]-softLimit[0])/2 +  ((analogRead(sensorPin)-512)*2));
      positions[3] = (softLimit[1]-softLimit[0])/2 +  ((analogRead(sensorPin)-512)*2); //232-792 is 560 steps centered about 512
      //positions[3] = (digitalRead(4)*500);
      //positions[3] = (digitalRead(4)*500);
      //steppers.errorComp();

      stepper_4.errorComp(stepper_4.currentPosition() - stepper_4.readEncoder());
      //if(abs(positions[3]-oldPos)>5){
      steppers.moveTo(positions);
        
        /*
              Serial.print("Encoder is: ");
              Serial.print(stepper_4.readEncoder());
              Serial.print("   ");
              Serial.print("Position is: ");
              Serial.println(stepper_4.currentPosition());
            */
            //note: steppers.run() is nonblocking, but my loop time is too slow
            // might be worth managing this by only updating position at like 1hz with a millis call/check in this loop
        //steppers.runSpeedToPosition(); //test this vs runSpeedToPosition... should account for missed steps and do calc to figure out max allowable speed
        
      //}
      long tn = millis();
        while (millis() < tn+10){
          steppers.run();
        }

      Serial.println(micros()-startTime);
      startTime = micros();
      //delayMicroseconds(48);
      //delay(10);
      oldPos = positions[3];
      
}

//ISR(TIMER1_COMPA_vect){
   //steppers.errorComp();
   //positions[3] = (softLimit[1]-softLimit[0])/2 +  analogRead(sensorPin);
//}
