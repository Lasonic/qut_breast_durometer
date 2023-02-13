
// sketch is intended for Arduino Mega

//======================================================================================================================================================
//||                                                          HARDWARE ASSIGNMENT                                                                     ||
//======================================================================================================================================================

//define hardware assignment for user input
#define PushButton1 22
#define PushButton2 24
#define PushButton3 26
#define PushButton4 28

//define hardware assignment for linear actuator feedback
#define EncoderCHA 2
#define EncoderCHB 3

//define hardware assignment for linear actuator control
#define PWMtoHbridge 5
#define ExtendActuatorCommand 6
#define RetractActuatorCommand 7

//define hardware assignment for system status
#define redLED 34
#define yellowLED 32
#define greenLED 30
#define speaker 4

//define hardware assignment for load cell
#define loadCell A0


//======================================================================================================================================================
//||                                                            GLOBAL VARIABLES                                                                      ||
//======================================================================================================================================================


//global variables
int PushButton1_Status = 0;
int PushButton2_Status = 0;
int PushButton3_Status = 0;
unsigned int motorSpeed = 125;
float AppliedForce = 0.0;
const int pulsesPerMM = 100;    //this value will change dependent on actuator encoder specification.
volatile long pulseCounts = 0;
volatile float linearTravel = 0.0;

//======================================================================================================================================================
//||                                                                 SETUP                                                                            ||
//======================================================================================================================================================

void setup() {
pinMode(PushButton1, INPUT);
pinMode(PushButton2, INPUT);
pinMode(PushButton3, INPUT);
pinMode(PushButton4, INPUT);
pinMode(EncoderCHA, INPUT);
pinMode(EncoderCHB, INPUT);
pinMode(PWMtoHbridge, OUTPUT);
pinMode(ExtendActuatorCommand, OUTPUT);
pinMode(RetractActuatorCommand, OUTPUT);
pinMode(redLED, OUTPUT);
pinMode(yellowLED, OUTPUT);
pinMode(greenLED, OUTPUT);
pinMode(speaker, OUTPUT);

//set up interrupts
attachInterrupt(digitalPinToInterrupt(2), ISR_ReadEncoder, FALLING);

//initialise serial connection
Serial.begin(9600);

}

//======================================================================================================================================================
//||                                                                 MAIN PROGRAM                                                                     ||
//======================================================================================================================================================

void loop() {
  // update the global variable for motorspeed via the serial port
      while(Serial.available() > 0){
    motorSpeed = Serial.parseInt();
  }

  PushButton3_Status = digitalRead(PushButton3);
  if(PushButton3_Status == LOW){
    StopActuator();
  }
 do{
  //read status of digital inputs 
  PushButton1_Status = digitalRead(PushButton1);
  PushButton2_Status = digitalRead(PushButton2);
  PushButton3_Status = digitalRead(PushButton3);

  //read status of analog inputs
  AppliedForce = analogRead(loadCell);
   
  if(PushButton1_Status == HIGH && PushButton2_Status == LOW){
    RunActuatorForward();
}
if(PushButton1_Status == LOW && PushButton2_Status == HIGH){
  RunActuatorReverse();
}
Serial.print("TRAVEL: ");
Serial.print(linearTravel);
Serial.print("  LOAD: ");
Serial.println(AppliedForce);
}
while (PushButton3_Status != HIGH);
StopActuator();
}

//======================================================================================================================================================
//||                                                                 SUB-ROUTINES                                                                     ||
//======================================================================================================================================================

void RunActuatorForward(void){
  digitalWrite(greenLED, HIGH);
  digitalWrite(yellowLED, LOW);
  digitalWrite(redLED, LOW);
  digitalWrite(RetractActuatorCommand, LOW);
  digitalWrite(ExtendActuatorCommand, HIGH);
  analogWrite(PWMtoHbridge, motorSpeed);
}
void RunActuatorReverse(void){

  digitalWrite(yellowLED, HIGH);
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(ExtendActuatorCommand, LOW);
  digitalWrite(RetractActuatorCommand, HIGH);
  analogWrite(PWMtoHbridge, motorSpeed);
}
void StopActuator(void){
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW);
  digitalWrite(yellowLED, LOW);
  digitalWrite(RetractActuatorCommand, LOW);
  digitalWrite(ExtendActuatorCommand, LOW);
  analogWrite(PWMtoHbridge, 0);
}

//======================================================================================================================================================
//||                                                         INTERRUPT SERVICE ROUTINES                                                               ||
//======================================================================================================================================================

void ISR_ReadEncoder(void){
int b = digitalRead(EncoderCHB);
int increment = 0;
if(b > 0){
  increment = 1;
}
else{
  increment = -1;
}
pulseCounts = pulseCounts + increment;

//calculate the linear travel based on pulses
linearTravel = pulseCounts / pulsesPerMM;
}
