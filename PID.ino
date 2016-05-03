
void updatePID(float restAngle, float offset, float turning, float dt) {
  /* Brake */
  if (steerStop) {
    int32_t wheelPosition = getWheelsPosition();
    int32_t positionError = wheelPosition - targetPosition;
    if (cfg.backToSpot) {
      if (abs(positionError) > zoneA) // Inside zone A
        restAngle -= (float)positionError / positionScaleA;
      else if (abs(positionError) > zoneB) // Inside zone B
        restAngle -= (float)positionError / positionScaleB;
      else if (abs(positionError) > zoneC) // Inside zone C
        restAngle -= (float)positionError / positionScaleC;
      else // Inside zone D
        restAngle -= (float)positionError / positionScaleD;
    } else {
      if (abs(positionError) < zoneC)
        restAngle -= (float)positionError / positionScaleD;
      else
        targetPosition = wheelPosition;
    }
    restAngle -= (float)wheelVelocity / velocityScaleStop;

    restAngle = constrain(restAngle, cfg.targetAngle - 10, cfg.targetAngle + 10); // Limit rest Angle
  }
  /* Drive forward and backward */
  else {
    if ((offset > 0 && wheelVelocity < 0) || (offset < 0 && wheelVelocity > 0) || offset == 0) // Scale down offset at high speed - wheel velocity is negative when driving forward and positive when driving backward
      offset += (float)wheelVelocity / velocityScaleMove; // We will always compensate if the offset is 0, but steerStop is not set
    restAngle -= offset;
  }

  restAngle = constrain(restAngle, lastRestAngle - 1, lastRestAngle + 1); // Don't change restAngle with more than 1 degree in each loop
  lastRestAngle = restAngle;

  /* Update PID values */
  float error = restAngle - pitch;
  float pTerm = cfg.P * error;
  iTerm += cfg.I * 100.0f * error * dt; // Multiplication with Ki is done before integration limit, to make it independent from integration limit value
  iTerm = constrain(iTerm, -100.0f, 100.0f); // Limit the integrated error - prevents windup
  float dTerm = (cfg.D / 100.0f) * (error - lastError) / dt;
  lastError = error;
  float PIDValue = pTerm + iTerm + dTerm;

  /* Steer robot sideways */
  if (turning < 0) { // Left
    turning += abs((float)wheelVelocity / velocityScaleTurning); // Scale down at high speed
    if (turning > 0)
      turning = 0;
  }
  else if (turning > 0) { // Right
    turning -= abs((float)wheelVelocity / velocityScaleTurning); // Scale down at high speed
    if (turning < 0)
      turning = 0;
  }

  float PIDLeft = PIDValue + turning;
  float PIDRight = PIDValue - turning;

  PIDLeft *= cfg.leftMotorScaler; // Compensate for difference in some of the motors
  PIDRight *= cfg.rightMotorScaler;

  //sprintf(serBuffer, "PIDLeft:%i PIDRight:%i", PIDLeft, PIDRight);
  //Serial.println(serBuffer); delay(250);

  // Print PID Variables pT, iT, dT
  sprintf(lcdBuffer,"pT:%s  iT:%s ", dtostrf(pTerm, 5 ,0 ,buf1), dtostrf(iTerm ,5 ,0 ,buf2));
  setcursorLCD(1,0); printLCDPanel(lcdBuffer, (uint8_t)sizeof(lcdBuffer));

  //Print PIDAngles to LCD
  sprintf(lcdBuffer,"dT:%s  pV:%s ",dtostrf(dTerm, 5, 0, buf1),dtostrf(PIDValue, 5, 0, buf2));
  setcursorLCD(2,0); printLCDPanel(lcdBuffer,(uint8_t)sizeof(lcdBuffer));

  /* Set PWM Values */
  if (PIDLeft >= 0)
    moveMotor(left, forward, PIDLeft);
  else
    moveMotor(left, backward, -PIDLeft);
  if (PIDRight >= 0)
    moveMotor(right, forward, PIDRight);
  else
    moveMotor(right, backward, -PIDRight);
}
