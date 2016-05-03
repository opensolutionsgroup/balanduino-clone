
#ifndef _balancingrobot2_h_
#define _balancingrobot2_h_
 
#if ARDUINO < 156 // Make sure the newest version of the Arduino IDE is used
#error "Please update the Arduino IDE to the newest version: http://arduino.cc/en/Main/Software"
#endif
 
#include <stdint.h> // Needed for uint8_t, uint16_t etc.
 
/* Firmware Version Information */
const char *version = "1.1.0";
const uint8_t eepromVersion = 3; // EEPROM version - used to restore the EEPROM values if the configuration values have changed
 
bool sendIMUValues, sendData, sendSettings, sendInfo, sendStatusReport, sendPIDValues, sendPairConfirmation, sendKalmanValues; // Used to send out different values via Bluetooth
 
/* Used to make commands more readable */
enum Command {
  updatePS3,
  updatePS4,
  updateWii,
  updateXbox,
  updateSpektrum,
  stop,
  forward,
  backward,
  left,
  right,
  imu,
  joystick,
} lastCommand; // This is used set a new targetPosition
 
#define spektrumBindPin P0 // Pin used to bind with the Spektrum satellite receiver - you can use any pin while binding, but you should connect it to RX0 afterwards
#define LED MAKE_PIN(LED_BUILTIN) // LED_BUILTIN is defined in pins_arduino.h in the hardware add-on
 
volatile int32_t leftCounter = 0, leftCounterRaw = 0;
volatile int32_t rightCounter = 0, rightCounterRaw = 0;
 
static float batteryVoltage; // Measured battery level
static uint8_t batteryCounter; // Counter used to check if it should check the battery level
 
bool ledState; // Last state of the built in LED
 
// This struct will store all the configuration values
typedef struct {
  float P, I, D; // PID variables
  float targetAngle; // Resting angle of the robot
  uint8_t backToSpot; // Set whenever the robot should stay in the same spot
  uint8_t controlAngleLimit; // Set the maximum tilting angle of the robot
  uint8_t turningLimit; // Set the maximum turning value
  float Qangle, Qbias, Rmeasure; // Kalman filter values
  float accXzero, accZzero; // Accelerometer zero values
  float leftMotorScaler, rightMotorScaler;
  bool bindSpektrum;
} cfg_t;
 
extern cfg_t cfg;
 
/* EEPROM Address Definitions */
const uint8_t initFlagsAddr = 0; // Set the first three bytes to the EEPROM version
const uint8_t configAddr = 1; // Save the configuration starting from this location
 
float lastRestAngle; // Used to limit the new restAngle if it's much larger than the previous one
 
/* IMU Data */
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t  accXYZ[3], gyroXYZ[3];
float gyroYzero;
 
uint8_t i2cBuffer[8]; // Buffer for I2C data
char lcdBuffer[24], serBuffer[40], buf1[6], buf2[6], buf3[6], buf4[6];
 
// Results
static float accAngle, gyroRate, gyroAngle;
static float pitch, temp;
 
static float lastError; // Store last angle error
static float iTerm; // Store iTerm
 
/* Used for timing */
static uint32_t kalmanTimer; // Timer used for the Kalman filter
static uint32_t pidTimer; // Timer used for the PID loop
static uint32_t encoderTimer; // Timer used used to determine when to update the encoder values
static uint32_t imuTimer; // This is used to set a delay between sending IMU values
static uint32_t reportTimer; // This is used to set a delay between sending report values
static uint32_t ledTimer; // Used to update the LEDs to indicate battery level on the PS3, Wii and Xbox controllers
static uint32_t blinkTimer; // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request
 
/* Used to rumble controllers upon connection */
static bool ps3RumbleEnable, wiiRumbleEnabled, ps4RumbleEnabled; // These are used to turn rumble off again on the Wiimote and PS4 controller and to turn on rumble on the PS3 controller
static bool ps3RumbleDisable, xboxRumbleDisable; // Used to turn rumble off again on the PS3 and Xbox controller
 
static bool steerStop = true; // Stop by default
static bool stopped; // This is used to set a new target position after braking
 
static bool layingDown = true; // Use to indicate if the robot is laying down
 
static float targetOffset = 0.0f; // Offset for going forward and backward
static float turningOffset = 0.0f; // Offset for turning left and right
 
static char dataInput[30]; // Incoming data buffer
static bool bluetoothData; // True if data received is from the Bluetooth connection
static float sppData1, sppData2; // Data send via SPP connection
 
static bool commandSent = false; // This is used so multiple controller can be used at once
 
static uint32_t receiveControlTimer;
static const uint16_t receiveControlTimeout = 500; // After how long time should it should prioritize the other controllers instead of the serial control
 
static int32_t lastWheelPosition; // Used to calculate the wheel velocity
static int32_t wheelVelocity; // Wheel velocity based on encoder readings
static int32_t targetPosition; // The encoder position the robot should be at
 
// Variables used for Spektrum receiver
extern uint16_t rcValue[]; // Channel values
static bool spekConnected; // True if spektrum receiver is connected
static uint32_t spekConnectedTimer; // Timer used to check if the connection is dropped
 
#define RC_CHAN_THROTTLE 0
#define RC_CHAN_ROLL     1
#define RC_CHAN_PITCH    2
#define RC_CHAN_YAW      3
#define RC_CHAN_AUX1     4
#define RC_CHAN_AUX2     5
#define RC_CHAN_AUX3     6
#if (SPEKTRUM == 2048) // 8 channels
#define RC_CHAN_AUX4     7
#endif
 
static const uint16_t zoneA = 8000;
static const uint16_t zoneB = 4000;
static const uint16_t zoneC = 1000;
static const float positionScaleA = 600.0f; // One resolution is 928 pulses per encoder
static const float positionScaleB = 800.0f;
static const float positionScaleC = 1000.0f;
static const float positionScaleD = 500.0f;
static const float velocityScaleMove = 70.0f;
static const float velocityScaleStop = 60.0f;
static const float velocityScaleTurning = 70.0f;
 
// Function prototypes
// We use inline to eliminates the size and speed overhead of calling and returning from a function that is only used once.
static inline void readSPPData();
static inline void readUsb();
static void updateLEDs();
static void onInitPS3();
static void onInitPS4();
static void onInitWii();
static void onInitXbox();
static void steer(Command command);
static float scale(float input, float inputMin, float inputMax, float outputMin, float outputMax);
 
static inline bool checkInitializationFlags();
static inline void readEEPROMValues();
static void updateConfig();
static void restoreEEPROMValues();
 
static inline void updatePID(float restAngle, float offset, float turning, float dt);
static void moveMotor(Command motor, Command direction, float speedRaw);
static void stopMotor(Command motor);
static inline void setPWM(Command motor, uint16_t dutyCycle);
static void stopAndReset();
// On newer versions of the PCB these two functions are only used in one place, so they will be inlined by the compiler.
static inline void leftEncoder();
static inline void rightEncoder();
static int32_t readLeftEncoder();
static int32_t readRightEncoder();
int32_t getWheelsPosition();
 
static inline void bindSpektrum();
static void readSpektrum(uint8_t input);
 
static inline void checkSerialData();
static void printMenu();
static inline void calibrateMotor();
static void testMotorSpeed(float *leftSpeed, float *rightSpeed, float leftScaler, float rightScaler);
static inline void calibrateAcc();
static inline void printValues();
static void setValues(char *input);
static inline bool calibrateGyro();
static bool checkMinMax(int16_t *array, uint8_t length, int16_t maxDifference);
 
float getVolts();
void initLCDPanel();
void setcursorLCD(uint8_t row, uint8_t col);
uint8_t printLCDPanel(char *buffer, uint8_t size);
uint8_t bufferFreeBytes();
 
uint8_t i2cWrite(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data, bool sendStop);
uint8_t i2cWrite(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data, uint8_t length, bool sendStop);
uint8_t i2cRead(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data, uint8_t nbytes);
 
/* I2C Bus Addresses */
const uint8_t gyroAddress = 0x68;
const uint8_t accAddress = 0x53;
const uint8_t motorAddress = 0x58;
const uint8_t lcdAddress = 0x63;

 
// MD25 Registers
#define CMD                 0x10
#define SPEED1              0x00
#define SPEED2              0x01
#define SOFTWAREREG         0x0D
#define ENCODERONE          0x02                              // Byte to read motor encoder 1
#define ENCODERTWO          0x06                              // Byte to read motor encoder 2
#define VOLTREAD            0x0A                              // Byte to read battery volts
#define RESETENCODERS       0x20
 
// LCD05 Registers
#define REG_COMMAND 0x00
#define REG_KEYPADLOW 0x01
#define REG_KEYPADHIGH 0x02
#define REG_VERSION 0x03
 
// LCD05 Commands
#define LCD_NOOP 0x00
#define LCD_CURSORHOME 0x01
#define LCD_CURSORPOS 0x02
#define LCD_CURSORPOSXY 0x03
#define LCD_CURSOROFF 0x04
#define LCD_CURSORON 0x05
#define LCD_CURSORBLINK 0x06
#define LCD_BACKSPACE 0x08
#define LCD_TAB 0x09
#define LCD_CURSORDOWN 0x0A
#define LCD_CURSORUP 0x0B
#define LCD_CLEARDISPLAY 0x0C
#define LCD_LINEFEED 0x0D
#define LCD_CLEARCOLUMN 0x11
#define LCD_TABSET 0x12
#define LCD_BACKLIGHTON 0x13
#define LCD_BACKLIGHTOFF 0x14
#define LCD_CUSTOMCHAR 0x1B
#define BUFFER_LENGTH 100
 
#endif
