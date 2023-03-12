// libraries and global variables

#include <SPI.h>  // SPI library
#include <Wire.h> // I2C library

/* OLED display setup */
#include <Adafruit_GFX.h> // Adafruit GFX library
#include <Adafruit_SSD1306.h> // Adafruit SSD1306 library
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/*load cell*/
#include <DFRobot_HX711_I2C.h>
//DFRobot_HX711_I2C MyScale(&Wire,/*addr=*/0x64);
DFRobot_HX711_I2C MyScale;

/* Tic Stepper Controller */
#include <Tic.h>  // Tic Stepper Controller library
TicI2C tic; // Initialise I2C communication with tic

float Weight = 0;
// targetPosition in steps. Default stepping mode: Full step. 1 step = 0.0018mm for P8-50-165-3-ST linear actuator

///////////////////
//  USER SETTINGS//
///////////////////
int32_t targetPosition = 4000; // Target position in steps. Refer to the conversion to mm. 
uint32_t speed = 1000000; // Target speed in steps per 10000 seconds

// Other variables //
int menu_flag = 0;
bool selectPositionFlag = true;
bool selectVelocityFlag = false;
bool startTestFlag = true;
bool inProgress = false;
bool positionSerialMessage = true;
bool velocitySerialMessage = false;
bool displayCurrentPosFlag = false;
int button = 7;


void setup()
{
  // Display setup. If display is not found, the program will display error message in Serial monitor.
  Serial.begin(9600); // Start Serial communication
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000);  // Pause for 2 seconds
  display.clearDisplay();  // Clear the buffer

  // Initialise HX711 amplifier. If the amplifier is not found, the program will display error message in Serial monitor.
  while (!MyScale.begin()) {
    Serial.println("The initialization of the chip is failed, please confirm whether the chip connection is correct");
    delay(1000);
  }
  //// Set the calibration weight when the weight sensor module is automatically calibrated (g)
  MyScale.setCalWeight(100);  // Default value
  // Set the trigger threshold (G) for automatic calibration of the weight sensor module. When only the weight of the object on the scale is greater than this value, the module will start the calibration process
  // This value cannot be greater than the calibration weight of the setCalWeight() setting
  MyScale.setThreshold(30);
  // Obtain the calibration value. The accurate calibration value can be obtained after the calibration operation is completed
  Serial.print("the calibration value of the sensor is: ");
  Serial.println(MyScale.getCalibration());
  MyScale.setCalibration(MyScale.getCalibration());
  delay(1000);

  // Start I2C
  Wire.begin();

  // Give the Tic some time to start up.
  delay(20);

  // Set the Tic's current position to 0, so that when we command
  // it to move later, it will move a predictable amount.

  tic.setMaxSpeed(speed); // Max speed is the speed specified by the user. The actuator will run at max speed.
  tic.haltAndSetPosition(0);

  // Tells the Tic that it is OK to start driving the motor.  The
  // Tic's safe-start feature helps avoid unexpected, accidental
  // movement of the motor: if an error happens, the Tic will not
  // drive the motor again until it receives the Exit Safe Start
  // command.  The safe-start feature can be disbled in the Tic
  // Control Center.
  tic.setStepMode(TicStepMode::Full);
  tic.exitSafeStart();
  
}

// Sends a "Reset command timeout" command to the Tic.  We must
// call this at least once per second, or else a command timeout
// error will happen.  The Tic's default command timeout period
// is 1000 ms, but it can be changed or disabled in the Tic
// Control Center.
void resetCommandTimeout()
{
  tic.resetCommandTimeout();
}

// Polls the Tic, waiting for it to reach the specified target
// position.  Note that if the Tic detects an error, the Tic will
// probably go into safe-start mode and never reach its target
// position, so this function will loop infinitely.  If that
// happens, you will need to reset your Arduino.
void waitForPosition(int32_t targetPosition)
{
  displayCurrentPosFlag = true;
  do
  {
    display.setTextSize(1);               // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);  // Draw white text
    display.clearDisplay();
    display.setCursor(0, 0);              // Set cursor to 0,0
    display.print("weight,vel,pos:");     // Display message
    display.setCursor(0, 20);             // Set cursor to 0,20
    // Display values on the serial monitor
    // Convert weight to N
    display.print(MyScale.readWeight()*0.00981, DEC);;  // Display weight in Newtons 

    display.setCursor(0, 30);   // Set cursor to 0,30
    // Default velocity: steps per 10000 seconds
    // Convert to mm/s
    display.print(tic.getCurrentVelocity()*0.0018/10000, DEC);;  // Display velocity in mm/s
    // Actuonix P8 stepper - 0.0018mm per step
    // Convert to display mm 
    display.setCursor(0, 40); // Set cursor to 0,40
    display.print(tic.getCurrentPosition()*0.0018, DEC);;  // Display current position in mm
    display.display();  // Display all previously set messages
    
    // Serial display
    // TIME(ms),WEIGHT(N),SPEED(mm/s),POSITION(mm)
    Serial.print(millis());
    Serial.print(",");
    Serial.print(MyScale.readWeight()*0.00981);
    Serial.print(",");
    Serial.print(tic.getCurrentVelocity()*0.0018/10000);
    Serial.print(",");
    Serial.println(tic.getCurrentPosition()*0.0018);
    delay(10);
    resetCommandTimeout();    
  } while (tic.getCurrentPosition() != targetPosition);
}

void loop()
{
  // If startTestFlag is true, continue.
  if(startTestFlag == true){
  tic.setTargetPosition(targetPosition); /* Go to the target position. Here, set 'targetPosition' to negative value
  to move in the opposite direction. The actuator does not have homing functionality. Once it has moved 10mm forward,
  it needs to then be manually moved -10mm to reach the home position. Use Tic control centre application if you
  run into problems. 
  */
  waitForPosition(targetPosition);  //Wait for the actuator to reach the target position

  startTestFlag = false;
  }
}
