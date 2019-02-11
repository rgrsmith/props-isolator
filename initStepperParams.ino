void initStepperParams() {

  #define MS1_1 22
  #define MS2_1 23
  #define SLP_1 25
  
  #define MS1_2 27
  #define MS2_2 28
  #define SLP_2 30
  
  #define MS1_3 37 // actually 4
  #define MS2_3 38
  #define SLP_3 40
  
  #define MS1_4 32 // actually 3
  #define MS2_4 33
  #define SLP_4 35
  
  // ----------------------  Initialize Motor Driver Pins   ----------------------
  pinMode(MS1_1, OUTPUT);
  pinMode(MS2_1, OUTPUT);
  pinMode(SLP_1, OUTPUT);
  digitalWrite(MS1_1, LOW);
  digitalWrite(MS2_1, HIGH);
  digitalWrite(SLP_1, HIGH);

  pinMode(MS1_2, OUTPUT);
  pinMode(MS2_2, OUTPUT);
  pinMode(SLP_2, OUTPUT);
  digitalWrite(MS1_2, LOW);
  digitalWrite(MS2_2, HIGH);
  digitalWrite(SLP_2, HIGH);
  
  pinMode(MS1_3, OUTPUT);
  pinMode(MS2_3, OUTPUT);
  pinMode(SLP_3, OUTPUT);
  digitalWrite(MS1_3, LOW);
  digitalWrite(MS2_3, HIGH);
  digitalWrite(SLP_3, HIGH);

  pinMode(MS1_4, OUTPUT);
  pinMode(MS2_4, OUTPUT);
  pinMode(SLP_4, OUTPUT);
  digitalWrite(MS1_4, LOW);
  digitalWrite(MS2_4, HIGH);
  digitalWrite(SLP_4, HIGH);
  // --------------------------------------------------------------------------
  
  // ----------------------  Setup Stepper Params   ---------------------------
  stepper_1.setEnablePin(26);
  stepper_1.setPinsInverted(true, false, true);
  stepper_1.enableOutputs();
  stepper_1.setCurrentPosition(0);
  stepper_1.setMaxSpeed(10000000000);
  stepper_1.setAcceleration(800000);
  //stepper_1.setEncoder(45);
  //stepper_1.moveTo(0);

  stepper_2.setEnablePin(31);
  stepper_2.setPinsInverted(true, false, true);
  stepper_2.enableOutputs();
  stepper_2.setCurrentPosition(0);
  stepper_2.setMaxSpeed(10000000000);
  stepper_2.setAcceleration(800000);

  stepper_3.setEnablePin(41); //actually 4
  stepper_3.setPinsInverted(true, false, true);
  stepper_3.enableOutputs();
  stepper_3.setCurrentPosition(0);
  stepper_3.setMaxSpeed(10000000000);
  stepper_3.setAcceleration(800000);

  stepper_4.setEnablePin(36); // actually 3
  stepper_4.setPinsInverted(true, false, true);
  stepper_4.enableOutputs();
  stepper_4.setCurrentPosition(0);
  stepper_4.setMaxSpeed(10000000000);
  stepper_4.setAcceleration(800000);
  // --------------------------------------------------------------------------
}
