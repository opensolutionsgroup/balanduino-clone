
float getTemp() {
    while (i2cRead(gyroAddress,0x1B,i2cBuffer,2));
    return (35 + (double)((((int)(i2cBuffer[0] << 8) | i2cBuffer[1]) + 13200) / 280.0));
}

float getVolts() {
    while (i2cRead(motorAddress,VOLTREAD,i2cBuffer,1));
    return ((double)i2cBuffer[0]/10);
}  

void moveMotor(Command motor, Command direction, float speedRaw) { // Speed is a value 0-100%
  uint16_t speed;
  if (speedRaw > 100.0f)
    speedRaw = 100.0f;
  if (motor == left) {
    if (direction == forward) {
      speed = map(speedRaw,0, 100, 128, 255);
      while(i2cWrite(motorAddress, SPEED1, speed, true));
    } 
    else if (direction == backward) {
      speed = map(speedRaw,0, 100, 128, 0);
      while (i2cWrite(motorAddress, SPEED1, speed, true));
    }
  } 
  else if (motor == right) {
    if (direction == forward) {
      speed = map(speedRaw, 0, 100, 128, 255);
      while (i2cWrite(motorAddress, SPEED2, speed, true));
    } 
    else if (direction == backward) {
      speed=map(speedRaw, 0, 100, 128, 0);
      while (i2cWrite(motorAddress, SPEED2, speed, true));
    }
  }
}

void stopMotor(Command motor) {
  if (motor == left)
    while (i2cWrite(motorAddress, SPEED1, 0x80, true)); // Sends value of 0x80 to stop motor
  else if (motor == right) 
    while (i2cWrite(motorAddress, SPEED2, 0x80, true)); // Sends value of 0x80 to stop motor
}

void stopAndReset() {
  stopMotor(left);
  stopMotor(right);
  lastError = 0;
  iTerm = 0;
  targetPosition = getWheelsPosition();
  lastRestAngle = cfg.targetAngle;
}

int32_t readLeftEncoder() {   // EMG30 30:1 Gearbox, 200 RPM, 360 Counts/Rev
    while (i2cRead(motorAddress, ENCODERONE, i2cBuffer, 4));
    int32_t counter = (int32_t)(i2cBuffer[0] | (i2cBuffer[1] << 8) | (i2cBuffer[2] << 8) | (i2cBuffer[3] << 8));
        
    if (counter != leftCounterRaw) {
      if (counter > leftCounterRaw) {
         leftCounter++ ;
      } else {
         leftCounter--; 
      }
      leftCounterRaw = counter;   
    }
  //return leftCounter;
    return leftCounterRaw;
}

int32_t readRightEncoder() {   // EMG30 30:1 Gearbox, 200 RPM, 360 Counts/Rev
      while (i2cRead(motorAddress, ENCODERTWO, i2cBuffer, 4));
      int32_t counter = (int32_t)(i2cBuffer[0] | (i2cBuffer[1] << 8) | (i2cBuffer[2] << 8) | (i2cBuffer[3] << 8));

    if (counter != rightCounterRaw) {
      if (counter > rightCounterRaw) {
         rightCounter++ ;
      } else {
         rightCounter--; 
      }
      rightCounterRaw = counter;
    }
  //return rightCounter;
    return rightCounterRaw;
}

int32_t getWheelsPosition() {
//return leftCounter + rightCounter;
  return leftCounterRaw + rightCounterRaw;
}

void resetEncoders() {                                           // This function resets the encoder values to 0
    while (i2cWrite(motorAddress, CMD, RESETENCODERS, true));    // Putting the value 0x20 to reset encoders
}
