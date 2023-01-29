#include <DFRobot_HX711_I2C.h>

//DFRobot_HX711_I2C MyScale(&Wire,/*addr=*/0x64);
DFRobot_HX711_I2C MyScale;

float Weight = 0;

void setup() {
  Serial.begin(9600);
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

void loop() {
  Weight = MyScale.readWeight();
  Serial.print("weight is: ");
  if(Weight > 0.5){
    Serial.print(Weight, 1);
  }
  else{
    Serial.print(0, 1);
  }
  Serial.println(" g");
  delay(200); 
}