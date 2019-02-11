void homeMot(AccelStepper stepper){
  int expectedPos = 0;
  //int movePos = 0;
  //Encoder1.clearEncoderCount();
  stepper.clearEncoder();
  //encoder1Reading = -Encoder1.readEncoder();
  encoder1Reading = stepper.readEncoder();
  expectedPos = stepper.currentPosition();
  while(abs(expectedPos - encoder1Reading)<20){
    //encoder1Reading = -Encoder1.readEncoder();
    encoder1Reading = stepper.readEncoder();
    expectedPos = stepper.currentPosition();
    stepper.setSpeed(1000);
    stepper.runToNewPosition(expectedPos-1);
    //expectedPos = expectedPos - 1;
    //Serial.print(expectedPos);
    //Serial.print("   ");
    //Serial.println(encoder1Reading);
  }
  /*
  for(int i=0; i<10; i++){
    actuator1.setPos(1, 80);
  }
 */
  
  stepper.setSpeed(100000);
    stepper.runToNewPosition(expectedPos+2);
    while(stepper.run()){
    }
  //Encoder1.clearEncoderCount();
  stepper.clearEncoder();
  stepper.setCurrentPosition(0);
  Serial.println("HOMED");
}
