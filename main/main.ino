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

float Weight = 0;
int menu_flag = 0;
int button = 7;

void setup() {

  // Displa
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
  //// Set the calibration weight when the weight sensor module is automatically calibrated (g)
  MyScale.setCalWeight(100);
  // Set the trigger threshold (G) for automatic calibration of the weight sensor module. When only the weight of the object on the scale is greater than this value, the module will start the calibration process
  // This value cannot be greater than the calibration weight of the setCalWeight() setting
  MyScale.setThreshold(30);
  // Obtain the calibration value. The accurate calibration value can be obtained after the calibration operation is completed
  Serial.print("the calibration value of the sensor is: ");
  Serial.println(MyScale.getCalibration());
  MyScale.setCalibration(MyScale.getCalibration());
  delay(1000);
}

void initial_message() {
  // Displays initial message at the start up
  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("Pawel Dan TEST"));
  display.display();
  delay(4000);
}
void main_menu_message() {
  // Displays initial message at the start up
  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("Press start for actuator test"));
  display.display();
}
void perform_actuator_test() {
  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("progress"));
  display.display();
  // Move to the X at X speed
}
void test_end_message() {
  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("finished"));
  display.display();
  // Move to the X at X speed
}


void loop() {
  // Display welcome message
  initial_message();
  main_menu_message();
  delay(4000);
  perform_actuator_test();
  Weight = MyScale.readWeight();
  while (Weight < 300) {
    Weight = MyScale.readWeight();
    display.clearDisplay();
    display.setTextSize(1);               // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);  // Draw white text
    display.setCursor(0, 0);              // Start at top-left corner
    display.println(Weight, DEC);;
    display.display();
  }
  delay(8000);
  test_end_message();
  delay(4000);
  /*
  while (menu_flag == 0){
  //wait for the button to be pressed
    if(button == true){
    menu_flag = 1;
    perform_actuator_test();
    // wait until actuator is not moving
    // display "finished"
    // menu_flag = 0;
    // main_menu_message();
    }  
  }
  */
}
