
/* Copyright (C) 2013-2015 Kristian Lauszus, TKJ Electronics. All rights reserved.
 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").
 Contact information
 -------------------
 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 This is the algorithm for the Balanduino balancing robot.
 It can be controlled by either an Android app or a computer application via Bluetooth.
 The Android app can be found at the following link: https://github.com/TKJElectronics/BalanduinoAndroidApp
 The Processing application can be found here: https://github.com/TKJElectronics/BalanduinoProcessingApp
 A dedicated Windows application can be found here: https://github.com/TKJElectronics/BalanduinoWindowsApp
 It can also be controlled by a PS3, PS4, Wii or a Xbox controller.
 Furthermore it supports the Spektrum serial protocol used for RC receivers.
 For details, see: http://balanduino.net/
*/

/* Use this to enable and disable the different options */
#define ENABLE_TOOLS
#define ENABLE_SPP
#define ENABLE_PS3
//#define ENABLE_PS4
//#define ENABLE_WII
//#define ENABLE_XBOX
#define ENABLE_ADK
//#define ENABLE_SPEKTRUM

#include "BalancingRobot2.h"
#include <Arduino.h> // Standard Arduino header
#include <Wire.h> // Official Arduino Wire library
#include <SPI.h> // Official Arduino SPI library

#ifdef ENABLE_ADK
#include <adk.h>
#endif
 
// These are all open source libraries written by Kristian Lauszus, TKJ Electronics
// The USB libraries are located at the following link: https://github.com/felis/USB_Host_Shield_2.0
#include <Kalman.h> // Kalman filter library - see: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

#ifdef ENABLE_XBOX
#include <XBOXRECV.h>
#endif
#ifdef ENABLE_SPP
#include <SPP.h>
#endif
#ifdef ENABLE_PS3
#include <PS3BT.h>
#endif
#ifdef ENABLE_PS4
#include <PS4BT.h>
#endif
#ifdef ENABLE_WII
#include <Wii.h>
#endif

// Create the Kalman library instance
Kalman kalman; // See https://github.com/TKJElectronics/KalmanFilter for source code

#if defined(ENABLE_SPP) || defined(ENABLE_PS3) || defined(ENABLE_WII) || defined(ENABLE_XBOX) || defined(ENABLE_ADK)
#define ENABLE_USB
USB Usb;  // This will take care of all USB communication
#endif

#ifdef ENABLE_ADK
// Implementation for the Android Open Accessory Protocol. Simply connect your phone to get redirected to the Play Store
ADK adk(&Usb, "TKJ Electronics", // Manufacturer Name
              "Balanduino", // Model Name
              "Android App for Balanduino", // Description - user visible string
              "0.6.3", // Version of the Android app
              "https://play.google.com/store/apps/details?id=com.tkjelectronics.balanduino", // URL - web page to visit if no installed apps support the accessory
              "1234"); // Serial Number - this is not used
#endif

#ifdef ENABLE_XBOX
XBOXRECV Xbox(&Usb); // You have to connect a Xbox wireless receiver to the Arduino to control it with a wireless Xbox controller
#endif

#if defined(ENABLE_SPP) || defined(ENABLE_PS3) || defined(ENABLE_PS4) || defined(ENABLE_WII)
#define ENABLE_BTD
#include <usbhub.h> // Some dongles can have a hub inside
USBHub Hub(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // This is the main Bluetooth library, it will take care of all the USB and HCI communication with the Bluetooth dongle
#endif

#ifdef ENABLE_SPP
SPP SerialBT(&Btd,"Balanduino","0000"); // The SPP (Serial Port Protocol) emulates a virtual Serial port, which is supported by most computers and mobile phones
#endif

#ifdef ENABLE_PS3
PS3BT PS3(&Btd); // This is the PS3 library. It supports all the three original controller: the Dualshock 3, Navigation and Move controller
#endif

#ifdef ENABLE_PS4
//PS4BT PS4(&Btd, PAIR); // You should create the instance like this if you want to pair with a PS4 controller, then hold PS and Share on the PS4 controller
// Or you can simply send "CPP;" to the robot to start the pairing sequence
// This can also be done using the Android or via the serial port
PS4BT PS4(&Btd); // The PS4BT library supports the PS4 controller via Bluetooth
#endif

#ifdef ENABLE_WII
WII Wii(&Btd); // The Wii library can communicate with Wiimotes and the Nunchuck and Motion Plus extension and finally the Wii U Pro Controller
//WII Wii(&Btd,PAIR); // You will have to pair with your Wiimote first by creating the instance like this and the press 1+2 on the Wiimote or press sync if you are using a Wii U Pro Controller
// Or you can simply send "CPW;" to the robot to start the pairing sequence
// This can also be done using the Android or via the serial port
#endif

void setup() {
  /* Initialize UART */
  Serial.begin(115200);

  /* Read the PID values, target angle and other saved values in the EEPROM */
  Serial.print("Initilizing EEPROM\n"); delay(250);
  if (!checkInitializationFlags()) {
    readEEPROMValues(); // Only read the EEPROM values if they have not been restored
#ifdef ENABLE_SPEKTRUM
    if (cfg.bindSpektrum) // If flag is set, then bind with Spektrum satellite receiver
      bindSpektrum();
#endif
  }
    
#ifdef ENABLE_USB
  Serial.print("Initilizing USB Controller\n"); delay(250);
  if (Usb.Init() == -1) { // Check if USB Host is working
    Serial.print(F("OSC did not start\n"));
    while (1); // Halt
  }
#endif

  /* Attach onInit function */
  // This is used to set the LEDs according to the voltage level and vibrate the controller to indicate the new connection
Serial.print("Initilizing Bluetooth Controller\n"); delay(250);
#ifdef ENABLE_PS3
  PS3.attachOnInit(onInitPS3);
#endif
#ifdef ENABLE_PS4
  PS4.attachOnInit(onInitPS4);
#endif
#ifdef ENABLE_WII
  Wii.attachOnInit(onInitWii);
#endif
#ifdef ENABLE_XBOX
  Xbox.attachOnInit(onInitXbox);
#endif

  /* Setup IMU */
  Serial.print("Initilizing I2C and IMU\n"); delay(250);
  Wire.begin();
//#if ARDUINO >= 157
//Wire.setClock(400000UL); // Set I2C frequency to 400kHz
//#else
//TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
//#endif

  /* Initilize LCD */
  Serial.print("Initilizing LCD Panel\n"); delay(250);
  initLCDPanel();
    
  Serial.print("Running Setup\n"); delay(250);
  
  while (i2cRead(gyroAddress,0x00,i2cBuffer,1));
  if (i2cBuffer[0] != gyroAddress) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading IMU sensor"));
    while (1); // Halt
  }
  
  while (i2cWrite(accAddress,0x2D,0x08, false));   // ADXL345: No AUTO_SLEEP
  while (i2cWrite(accAddress,0x31,0x00, false));   // ADXL345: +/-2g Full Range
  while (i2cWrite(gyroAddress,0x15,0x13, false));  // ITG3205: Gyro at 8KHz/(19+1)=400Hz sample rate
  while (i2cWrite(gyroAddress,0x16,0x00, true));   // ITG3205: Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  
  delay(100); // Wait for the sensor to get ready
  
  while(i2cRead(accAddress,0x32,i2cBuffer,6));
  int16_t accX = (i2cBuffer[0] | (i2cBuffer[1] << 8));
  int16_t accY = (i2cBuffer[2] | (i2cBuffer[3] << 8));
  int16_t accZ = (i2cBuffer[4] | (i2cBuffer[5] << 8));
  
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  //accAngle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  //accAngle = (atan2((float)accY - cfg.accYzero, (float)accZ - cfg.accZzero) + PI) * RAD_TO_DEG;  // Original
  accAngle = (atan2((float)accX - cfg.accXzero, (float)accZ - cfg.accZzero) + PI) * RAD_TO_DEG;  // BalancingRobot2  X=-Y, Y=X, Z=Z 
 
  kalman.setAngle(accAngle); // Set starting angle
  pitch = accAngle;
  gyroAngle = accAngle;
 
  /* Calibrate gyro zero value */
  // while (calibrateGyro()); // Run again if the robot is moved while calibrating

  pinMode(LED_BUILTIN,OUTPUT); // LED_BUILTIN is defined in pins_arduino.h in the hardware add-on
  stopAndReset(); // Turn off motors and reset different values

#ifdef ENABLE_TOOLS
  printMenu();
#endif
   
  /* Setup timing */  
  kalmanTimer = micros();
  pidTimer = kalmanTimer;
  encoderTimer = kalmanTimer;
  imuTimer = millis();
  ledTimer = imuTimer;
  blinkTimer = imuTimer;
}

void loop() {

#if defined(ENABLE_WII) || defined(ENABLE_PS4) // We have to read much more often from the Wiimote and PS4 controller to decrease latency
  bool readUSB = false;
#ifdef ENABLE_WII
  if (Wii.wiimoteConnected)
    readUSB = true;
#endif
#ifdef ENABLE_PS4
  if (PS4.connected())
    readUSB = true;
#endif
  if (readUSB)
    Usb.Task();
#endif
  
  while(i2cRead(accAddress,0x32,i2cBuffer,6));
  int16_t accX = (i2cBuffer[0] | (i2cBuffer[1] << 8));
  int16_t accY = (i2cBuffer[2] | (i2cBuffer[3] << 8));
  int16_t accZ = (i2cBuffer[4] | (i2cBuffer[5] << 8));
   
  while(i2cRead(gyroAddress,0x1D,i2cBuffer,6));
  int16_t gyroX = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
  int16_t gyroY = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
  int16_t gyroZ = ((i2cBuffer[4] << 8) | i2cBuffer[5]);

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
//accAngle = (atan2((float)accY - cfg.accYzero, (float)accZ - cfg.accZzero) + PI) * RAD_TO_DEG;  // Original
  accAngle = (atan2((float)accX - cfg.accXzero, (float)accZ - cfg.accZzero) + PI) * RAD_TO_DEG;  // BalancingRobot2  X=-Y, Y=X, Z=Z 
  
  //sprintf(serBuffer, "X:%2.2i Y:%2.2i Z:%2.2i accAngle:%s Down:%s", accX, accY, accZ, dtostrf(accAngle, 3, 0, buf1), layingDown?"True":"False");
  //Serial.println(serBuffer); delay(250);

  uint32_t timer = micros();
  // This fixes the 0-360 transition problem when the accelerometer angle jumps between 0 and 360 degrees
   if ((accAngle < 90 && pitch > 270) || (accAngle > 270 && pitch < 90)) {
     kalman.setAngle(accAngle);
     pitch = accAngle;
     gyroAngle = accAngle;
   } else {
     //float gyroRate =  ((float)gyroX - gyroXzero) / 131.0f; // Convert to deg/s Original
       float gyroRate = -((float)gyroY - gyroYzero) / 131.0f; // Convert to deg/s BalancingRobot2  X=-Y, Y=X, Z=Z 
       
       float dt = (float)(timer - kalmanTimer) / 1000000.0f;
       gyroAngle += gyroRate * dt; // Gyro angle is only used for debugging
       
       if (gyroAngle < 0 || gyroAngle > 360)
         gyroAngle = pitch; // Reset the gyro angle when it has drifted too much
       
       pitch = kalman.getAngle(accAngle, gyroRate, dt); // Calculate the angle using a Kalman filter

       //sprintf(serBuffer, "Pitch:%s accAngle:%s gyroRate:%s dt:%s", dtostrf(pitch, 3, 0, buf1), dtostrf(accAngle, 3, 0, buf2), dtostrf(gyroRate, 3, 0, buf3), dtostrf(dt, 3, 0, buf4));
       //Serial.println(serBuffer); delay(250);

       // Print Angle/Gyro/Kalman to LCD
       sprintf(lcdBuffer,"aY:%s gY:%s kY:%s", dtostrf(accAngle, 3, 0, buf1), dtostrf(gyroRate, 3, 0, buf2), dtostrf(pitch, 3, 0, buf3)); 
       setcursorLCD(0,0); printLCDPanel(lcdBuffer,(uint8_t)sizeof(lcdBuffer));
   }
   kalmanTimer = timer;
    
#if defined(ENABLE_WII) || defined(ENABLE_PS4) // We have to read much more often from the Wiimote and PS4 controller to decrease latency
  if (readUSB)
    Usb.Task();
#endif
  
  /* Drive motors */
  timer = micros();
  // If the robot is laying down, it has to be put in a vertical position before it starts balancing
  // If it's already balancing it has to be ±45 degrees before it stops trying to balance
  if ((layingDown && (pitch < cfg.targetAngle - 10 || pitch > cfg.targetAngle + 10)) || (!layingDown && (pitch < cfg.targetAngle - 45 || pitch > cfg.targetAngle + 45))) {
    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();
  } else {
    layingDown = false; // It's no longer laying down
    updatePID(cfg.targetAngle, targetOffset, turningOffset, (float)(timer - pidTimer) / 1000000.0f);
  }
  pidTimer = timer;

  /* Update encoders */
   timer = millis();
  if (timer - encoderTimer >= 100) { // Update encoder values every 100ms
    encoderTimer = timer;
    int32_t wheelPosition = getWheelsPosition();
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;
    //Serial.print(wheelPosition);Serial.print('\t');Serial.print(targetPosition);Serial.print('\t');Serial.println(wheelVelocity);
    if (abs(wheelVelocity) <= 40 && !stopped) { // Set new targetPosition if braking
      targetPosition = wheelPosition;
      stopped = true;
    }
    
    batteryCounter++;
    if (batteryCounter > 10) { // Measure battery every 1s
      batteryCounter = 0;
      batteryVoltage = getVolts();

      // Print Battery Voltage and CPU Temp to LCD
      sprintf(lcdBuffer,"Vb:%sV  Tp:%sC  ", dtostrf(getVolts(), 3 , 1, buf1), dtostrf(getTemp(), 3, 1, buf2));
      setcursorLCD(3,0); printLCDPanel(lcdBuffer, (uint8_t)sizeof(lcdBuffer));
    }
  }

  /* Read the Bluetooth dongle and send PID and IMU values */
#if defined(ENABLE_TOOLS) || defined(ENABLE_SPEKTRUM)
  checkSerialData();
#endif
#if defined(ENABLE_USB) || defined(ENABLE_SPEKTRUM)
  readUsb();
#endif
#if defined(ENABLE_TOOLS) || defined(ENABLE_SPP)
  printValues();
#endif

#ifdef ENABLE_BTD
  if (Btd.isReady()) {
    timer = millis();
    if ((Btd.watingForConnection && timer - blinkTimer > 1000) || (!Btd.watingForConnection && timer - blinkTimer > 100)) {
      blinkTimer = timer;
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState); // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request
    }
  } else if (ledState) { // The LED is on
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState); // This will turn it off
  }
#endif
}

