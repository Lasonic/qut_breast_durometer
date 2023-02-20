// libraries and global variables

#include <SPI.h>
#include <Wire.h>

/* display */
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
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
/* Tic Stepper Controller library */

#include <Tic.h>

TicI2C tic;
//---------------------------------Define variables---------------------------------------------//
float Weight = 0;
int targetPosition = 200;
int targetVelocity = 0;
int menu_flag = 0;
bool selectPositionFlag = true;
bool selectVelocityFlag = false;
bool startTestFlag = false;
bool inProgress = false;
bool positionSerialMessage = true;
bool velocitySerialMessage = false;
bool displayCurrentPosFlag = false;
int button = 7;


void setup()
{
  // Display
  Serial.begin(9600);
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
  // Clear the buffer
  display.clearDisplay();
  while (!MyScale.begin()) {
    Serial.println("The initialization of the chip is failed, please confirm whether the chip connection is correct");
    delay(1000);
  }
  // Set up I2C.
  Wire.begin();
  // Give the Tic some time to start up.
  delay(20);

  // Set the Tic's current position to 0, so that when we command
  // it to move later, it will move a predictable amount.
  tic.haltAndSetPosition(0);

  // Tells the Tic that it is OK to start driving the motor.  The
  // Tic's safe-start feature helps avoid unexpected, accidental
  // movement of the motor: if an error happens, the Tic will not
  // drive the motor again until it receives the Exit Safe Start
  // command.  The safe-start feature can be disbled in the Tic
  // Control Center.
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

// Delays for the specified number of milliseconds while
// resetting the Tic's command timeout so that its movement does
// not get interrupted by errors.
/*void delayWhileResettingCommandTimeout(uint32_t ms)
{
  uint32_t start = millis();
  do
  {
    display.clearDisplay();                           // Clear display
    display.setTextSize(1);                           // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);              // Draw white text
    display.setCursor(0, 0);                          // Start at top-left corner
    display.println(tic.getCurrentPosition(), DEC);;  // Get current stepper position (step count)
    display.display();                                // Display
    resetCommandTimeout();                            
  } while ((uint32_t)(millis() - start) <= ms);       // 
}
*/

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
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Current position and speed:");
    display.setCursor(0, 20);  
    display.print(tic.getCurrentPosition(), DEC);;
    display.setCursor(0, 30);
    display.print(tic.getCurrentVelocity(), DEC);;  
    display.display();
    delay(50);
    resetCommandTimeout();    
  } while (tic.getCurrentPosition() != targetPosition);
}

void loop()
{
  // Tell the Tic to move to targetPosition , and wait until it gets
  // there.
  tic.setTargetPosition(targetPosition);
  waitForPosition(targetPosition);
  
  // Tell the Tic to move to -targetPosition , and wait until it gets
  // there.
  tic.setTargetPosition(-targetPosition);
  waitForPosition(-targetPosition);
}
