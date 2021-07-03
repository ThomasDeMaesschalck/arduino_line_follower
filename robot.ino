#include <NewPing.h>
const int vSpeed = 130;     //forward driving speed MAX 255
const  int turn_speed = 50;   //turning speed MAX 255 

int checkKant = 0; //store the last turning position of the robot, used to optimize driving behaviour 
  
//L293 motorshield connection   
  const int motorA1      = 2; 
  const int motorAspeed  = 5;
  const int motorB1      = 8;
  const int motorBspeed  = 11;

//RGB LED config
const int RGBBlue = 7;
const int RGBGreen = 6;
const int RGBRed = 4;
int RGB1 = 0;
int RGB2 = 0;
int RGB3 = 0;

//left turning LED
const int linkerPinker = 12;

//right turning LED
const int rechterPinker = 10;

//ultrasonic config, used for obstacle detection
const int trigPin = 20;
const int echoPin = 21;
int distanceCm;
//second  ultrasonic sensor, used to detect a bridge on the obstacle course
const int trigPinBridge = 18;
const int echoPinBridge = 19;
int distanceCmBridge;

//NewPing library sonar config
#define SONAR_NUM 2
#define MAX_DISTANCE 200
#define PING_INTERVAL 66
unsigned long pingTimer[SONAR_NUM];
uint8_t currentSensor = 0;
unsigned int cm[SONAR_NUM];

NewPing sonar[SONAR_NUM] = {
  NewPing(trigPin, echoPin, MAX_DISTANCE),
  NewPing(trigPinBridge, echoPinBridge, MAX_DISTANCE)
};

//configuration of robot behaviour when a bridge is detected on the obstacle course
int ledStateBridge = LOW;
int RGBledStateBridge = LOW;
unsigned long previousMillisBridge = 0;
const long intervalBridge = 100;
bool onderDeBrugVertraging = false;

//obstacle detection buzzer settings
int buzzerState = LOW;
unsigned long previousMillisBuzzer = 0;

//obstacle dection LED blinking settings
unsigned long  previousMillisObstacle = 0;
const long intervalObstacle = 500;
int ledStateObstacle = LOW;
int RGBledStateObstacle = LOW;

//buzzer config
const int buzzer = 13;

//Sensor Connection
  const int left_sensor_pin = 33;
  const int middle_sensor_pin = 35;
  const int right_sensor_pin = 37;

  int left_sensor_state;
  int middle_sensor_state;
  int right_sensor_state;

 //color sensor left
#define S0Left 50
#define S1Left 48
#define S2Left 44
#define S3Left 42
#define ColorSensorOutLeft 46
#define OELeft 52
// Stores frequency read by the photodiodes
int redFrequencyLeft = 0;
int greenFrequencyLeft = 0;
int blueFrequencyLeft = 0;
String colorLeft;

 //color sensor right
#define S0Right 30
#define S1Right 32
#define S2Right 24
#define S3Right 26
#define ColorSensorOutRight 22
#define OERight 28
// Stores frequency read by the photodiodes
int redFrequencyRight = 0;
int greenFrequencyRight = 0;
int blueFrequencyRight = 0;
String colorRight;

//color recognition config
bool colorLeftRightAan = true;
unsigned long colorSensorDelay = 0;
  
void setup() {
 //Serial.begin(115200);

 pinMode(motorA1,OUTPUT); 
 pinMode(motorAspeed,OUTPUT);  
 pinMode(motorB1,OUTPUT); 
 pinMode(motorBspeed,OUTPUT); 
 pinMode(left_sensor_pin,INPUT);
 pinMode(middle_sensor_pin,INPUT);
 pinMode(right_sensor_pin,INPUT);
 pinMode(trigPin, OUTPUT);
 pinMode(echoPin, INPUT);
 pinMode(trigPinBridge, OUTPUT);
 pinMode(echoPinBridge, INPUT);
 pinMode(buzzer, OUTPUT);
 pinMode(RGBBlue, OUTPUT);
 pinMode(RGBGreen, OUTPUT);
 pinMode(RGBRed, OUTPUT);
 pinMode(linkerPinker, OUTPUT);
 pinMode(rechterPinker, OUTPUT);
 pinMode(S0Left, OUTPUT);
  pinMode(S1Left, OUTPUT);
  pinMode(S2Left, OUTPUT);
  pinMode(S3Left, OUTPUT);
  pinMode(OELeft, OUTPUT);
  pinMode(ColorSensorOutLeft, INPUT);
  pinMode(S0Right, OUTPUT);
  pinMode(S1Right, OUTPUT);
  pinMode(S2Right, OUTPUT);
  pinMode(S3Right, OUTPUT);
  pinMode(OERight, OUTPUT);
  pinMode(ColorSensorOutRight, INPUT);

//color recognition sensor configuration
  digitalWrite(S0Left,HIGH);
  digitalWrite(S1Left,LOW);
  digitalWrite(OELeft, LOW);
  digitalWrite(S0Right,HIGH);
  digitalWrite(S1Right,LOW);
  digitalWrite(OERight, LOW);
  
//Countdown procedure. On startup, the robot will stand still for five seconds. The RGB LEDs will blink in a yellow color.
RGB_color(200, 100, 0);
digitalWrite(linkerPinker, HIGH);
digitalWrite(rechterPinker, HIGH);
delay(500);
RGB_color(0, 0, 0);
digitalWrite(linkerPinker, LOW);
digitalWrite(rechterPinker, LOW);
  delay(500);
RGB_color(200, 100, 0);
digitalWrite(linkerPinker, HIGH);
digitalWrite(rechterPinker, HIGH);
  delay(500);
RGB_color(0, 0, 0);
digitalWrite(linkerPinker, LOW);
digitalWrite(rechterPinker, LOW);
  delay(500);
RGB_color(200, 100, 0);
digitalWrite(linkerPinker, HIGH);
digitalWrite(rechterPinker, HIGH);
  delay(500);
RGB_color(0, 0, 0);
digitalWrite(linkerPinker, LOW);
digitalWrite(rechterPinker, LOW);
  delay(500);
RGB_color(200, 100, 0);
digitalWrite(linkerPinker, HIGH);
digitalWrite(rechterPinker, HIGH);
  delay(500);
RGB_color(0, 0, 0);
digitalWrite(linkerPinker, LOW);
digitalWrite(rechterPinker, LOW);
  delay(500);
RGB_color(200, 100, 0);
digitalWrite(linkerPinker, HIGH);
digitalWrite(rechterPinker, HIGH);
  delay(500);
RGB_color(0, 0, 0);
digitalWrite(linkerPinker, LOW);
digitalWrite(rechterPinker, LOW);
  delay(500);

RGB1 = 0;
RGB2 = 255;
RGB3 = 0;
RGB_color(RGB1, RGB2, RGB3); //RGB LEDs turn green when countdown procedure is done


//NewPing library configuration
pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
 pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop() {  //this is the code that gets looped when the startup procedure is done

//read out of the three line following sensors
left_sensor_state = digitalRead(left_sensor_pin);
middle_sensor_state = digitalRead(middle_sensor_pin);
right_sensor_state = digitalRead(right_sensor_pin);

//sonar ping action
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  
//Robot behaviour changes when an object above the robot is detected.
if (distanceCmBridge > 0 && distanceCmBridge <50)
{ 
  OnderDeBrug();
 onderDeBrugVertraging = true;
  }
  else {
    digitalWrite(linkerPinker, LOW); 
    digitalWrite(rechterPinker, LOW);  
    onderDeBrugVertraging = false;
    }
  
//Turn color detection back on if it's deactivated. When a color is detected, the sensors are switched off for eight seconds so the same action doesn't get triggered multiple times
if (!colorLeftRightAan && ((millis() - colorSensorDelay) >= 8000))   
{
colorLeftRightAan = true;
}


//Retrieve colors from the color sensors, unless color detection is switched off
if (colorLeftRightAan == true) 
{
ColorLeft();
ColorRight();
}
else 
{
colorLeft = "OTHER";
colorRight = "OTHER";
}


//Turn robot counterclockwise when a red color is detected left of the robot 
if(colorLeft == "RED")
{
ColorLeftRed();
}

//Turn robot clockwise when a green color is detected right of the robot
if(colorRight == "GREEN")
{
ColorRightGreen();
}

//Robot needs to change RGB LED behaviour when a horizontal white line is detected
if(colorRight == "WHITE" && middle_sensor_state == LOW && colorLeft == "WHITE")
{ 
  RGB1 = 155;
  RGB2 = 0;
  RGB3 = 100;
RGB_color(RGB1, RGB2, RGB3);
}

//CASE 1: Forward driving
if(left_sensor_state == HIGH && middle_sensor_state == LOW && right_sensor_state == HIGH)  //HIGH is zwart
{
if (distanceCm > 0 && distanceCm < 48 ) 
{
ObstakelRijgedrag(115, 115);
}
if (onderDeBrugVertraging) 
{ 
digitalWrite(motorA1,LOW);                    
digitalWrite(motorB1,LOW);
analogWrite (motorAspeed, 90);
analogWrite (motorBspeed, 90);
}
else
{
RGB_color(RGB1, RGB2, RGB3); 
  digitalWrite(motorA1,LOW);  //LOW is vooruit                     
  digitalWrite(motorB1,LOW);
  digitalWrite(buzzer, LOW);
  analogWrite (motorAspeed, 115);
  analogWrite (motorBspeed, 115);
}
  }

//CASE 2: Turn right
if(left_sensor_state == HIGH &&  middle_sensor_state == LOW && right_sensor_state == LOW)
{
  checkKant = 1; //laatste draai bijhouden

  digitalWrite (motorA1,LOW);                    
  digitalWrite (motorB1,LOW);

if (distanceCm > 0 && distanceCm < 40 ) 
{
ObstakelRijgedrag(turn_speed, vSpeed);
}

else
{
  analogWrite (motorAspeed, turn_speed);
  analogWrite (motorBspeed, vSpeed);

  }
}

//CASE3: Turn left

if(left_sensor_state == LOW && middle_sensor_state == LOW && right_sensor_state == HIGH)
{
 checkKant = 2; //laatste draai bijhoduen

  digitalWrite (motorA1,LOW);                      
  digitalWrite (motorB1,LOW);

if (distanceCm > 0 && distanceCm < 40 ) {
ObstakelRijgedrag(vSpeed, turn_speed);
  }

else
{
  analogWrite (motorAspeed, vSpeed);
  analogWrite (motorBspeed, turn_speed); 
  }
}

//CASE 4: Hard turn to the left
if(left_sensor_state == LOW && middle_sensor_state == HIGH && right_sensor_state == HIGH)
{
  digitalWrite (motorA1,LOW);                    
  digitalWrite (motorB1,HIGH);

  analogWrite (motorAspeed, 110);
  analogWrite (motorBspeed, 110);

  }

//CASE 5: Hard turn to the right
if(left_sensor_state == HIGH && middle_sensor_state == HIGH && right_sensor_state == LOW)
{
 
  digitalWrite (motorA1,HIGH);                    
  digitalWrite (motorB1,LOW);

  analogWrite (motorAspeed, 110); 
  analogWrite (motorBspeed, 110); 

  }
    
  //CASE 6: Sharp corners
if(left_sensor_state == HIGH && middle_sensor_state == HIGH && right_sensor_state == HIGH)
{ 

if (checkKant == 1){ //Sharp corner to the right

  for (uint8_t i = 0; i < SONAR_NUM; i++) pingTimer[i] = -1; //for the sonar NewPing timer
  analogWrite (motorAspeed, 0);
  analogWrite (motorBspeed, 0);

  delay (20);

  digitalWrite (motorA1,HIGH);                    
  digitalWrite (motorB1,LOW);

  analogWrite (motorAspeed, 140);
  analogWrite (motorBspeed, 140);

  
   do {
      right_sensor_state = analogRead(right_sensor_pin);
    } while(right_sensor_state == HIGH);

    
  delay(20);
  pingTimer[0] = millis(); //voor sonar NewPing timer
for (uint8_t i = 1; i < SONAR_NUM; i++) pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  }

 if (checkKant == 2){
  //Sharp corner to the left
  for (uint8_t i = 0; i < SONAR_NUM; i++) pingTimer[i] = -1;
  analogWrite (motorAspeed, 0);
  analogWrite (motorBspeed, 0);

  delay (20);

    digitalWrite (motorA1,LOW);                    
  digitalWrite (motorB1,HIGH);

  analogWrite (motorAspeed, 140);
  analogWrite (motorBspeed, 140);
  
   do {
      left_sensor_state = analogRead(left_sensor_pin);      
    } while(left_sensor_state == HIGH);

  delay(20);
  pingTimer[0] = millis();
for (uint8_t i = 1; i < SONAR_NUM; i++) pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  }
  
  checkKant = 0; //set this back to 0 after taking a corner code is done
  
  
  }
}

 void RGB_color(int red_light_value, int green_light_value, int blue_light_value) //method to change the RGB LED colors
 {
  analogWrite(RGBRed, red_light_value);
  analogWrite(RGBGreen, green_light_value);
  analogWrite(RGBBlue, blue_light_value);
}


void ColorLeft() //retrieve color from the left color recognition sensor
  {
  digitalWrite(S2Left,LOW);
  digitalWrite(S3Left,LOW);
  redFrequencyLeft = pulseIn(ColorSensorOutLeft, LOW);
  digitalWrite(S2Left,HIGH);
  digitalWrite(S3Left,HIGH);
  greenFrequencyLeft = pulseIn(ColorSensorOutLeft, LOW);
  digitalWrite(S2Left,LOW);
  digitalWrite(S3Left,HIGH);
  blueFrequencyLeft = pulseIn(ColorSensorOutLeft, LOW);
  if (redFrequencyLeft > 60  && redFrequencyLeft < 105   &&  greenFrequencyLeft >  45 && greenFrequencyLeft < 67    &&  blueFrequencyLeft > 50  && blueFrequencyLeft < 73)   colorLeft = "GREEN";
  else if (redFrequencyLeft > 15 && redFrequencyLeft < 30  &&  greenFrequencyLeft > 50 && greenFrequencyLeft < 95   &&  blueFrequencyLeft > 45 && blueFrequencyLeft < 75)  colorLeft = "RED";
  else if (redFrequencyLeft > 0 && redFrequencyLeft < 40  &&  greenFrequencyLeft > 0 && greenFrequencyLeft < 40   &&  blueFrequencyLeft > 0 && blueFrequencyLeft < 40)  colorLeft = "WHITE";
  else colorLeft = "OTHER";
//Serial.println(colorLeft);
  }

 void ColorRight() //retrieve color from the right color recognition sensor
 {
   digitalWrite(S2Right,LOW);
  digitalWrite(S3Right,LOW);
  redFrequencyRight = pulseIn(ColorSensorOutRight, LOW);
  digitalWrite(S2Right,HIGH);
  digitalWrite(S3Right,HIGH);
  greenFrequencyRight = pulseIn(ColorSensorOutRight, LOW);
  digitalWrite(S2Right,LOW);
  digitalWrite(S3Right,HIGH);
  blueFrequencyRight = pulseIn(ColorSensorOutRight, LOW);
  if (redFrequencyRight > 60  && redFrequencyRight < 105   &&  greenFrequencyRight >  45 && greenFrequencyRight < 67    &&  blueFrequencyRight > 50  && blueFrequencyRight < 73)   colorRight = "GREEN";
  else if (redFrequencyRight > 15 && redFrequencyRight < 30  &&  greenFrequencyRight > 50 && greenFrequencyRight < 95   &&  blueFrequencyRight > 45 && blueFrequencyRight < 75)  colorRight = "RED";
  else if (redFrequencyRight > 0 && redFrequencyRight < 40  &&  greenFrequencyRight > 0 && greenFrequencyRight < 40   &&  blueFrequencyRight > 0 && blueFrequencyRight < 40)  colorRight = "WHITE";
  else colorRight = "OTHER";
//Serial.println(colorRight);
  }

void ObstakelRijgedrag(int motorA, int motorB) //detect an obstacle on the course and perform the required behaviour
{

     if (distanceCm >37 && distanceCm <48 ) { //values depend on how far from the front of the robot the sensor is located
  analogWrite (motorAspeed, (motorA*0.85));
  analogWrite (motorBspeed, (motorB*0.85));

   unsigned long currentMillisBuzzer = millis(); //timer for the buzzer
  if (currentMillisBuzzer - previousMillisBuzzer >= 200) {
    previousMillisBuzzer = currentMillisBuzzer;
    if (buzzerState == LOW) {
      buzzerState = HIGH;
    } else {
      buzzerState = LOW;
    }
digitalWrite(buzzer, buzzerState);
}
  }

  if (distanceCm >27 && distanceCm <37) {
  analogWrite (motorAspeed, (motorA*0.85));
  analogWrite (motorBspeed, (motorB*0.85));

 unsigned long currentMillisBuzzer = millis();
  if (currentMillisBuzzer - previousMillisBuzzer >= 100) {
    previousMillisBuzzer = currentMillisBuzzer;
    if (buzzerState == LOW) {
      buzzerState = HIGH;
    } else {
      buzzerState = LOW;
    }
digitalWrite(buzzer, buzzerState);
}
}

 if (distanceCm >17 && distanceCm <27) {
 analogWrite (motorAspeed, (motorA*0.85));
 analogWrite (motorBspeed, (motorB*0.85));
   unsigned long currentMillisBuzzer = millis();
  if (currentMillisBuzzer - previousMillisBuzzer >= 50) {
    previousMillisBuzzer = currentMillisBuzzer;
    if (buzzerState == LOW) {
      buzzerState = HIGH;
    } else {
      buzzerState = LOW;
    }
digitalWrite(buzzer, buzzerState);
}
  }

if (distanceCm >= 0 && distanceCm <17 ) {
  analogWrite (motorAspeed, 0);
  analogWrite (motorBspeed, 0);
  digitalWrite(buzzer, HIGH);
    unsigned long currentMillisObstacle = millis();
  if (currentMillisObstacle - previousMillisObstacle >= intervalObstacle) {
    previousMillisObstacle = currentMillisObstacle;
    if (ledStateObstacle == LOW) {
      ledStateObstacle = HIGH;
    } else {
      ledStateObstacle = LOW;
    }
    if (RGBledStateObstacle == LOW) {
      RGB_color(200,100,0);
      RGBledStateObstacle = HIGH;
    } else {
      RGBledStateObstacle = LOW;
      RGB_color(0,0,0);
    }
digitalWrite(linkerPinker, ledStateObstacle);
digitalWrite(rechterPinker, ledStateObstacle);  
}
}
}

  void OnderDeBrug() { //robot behaviour when a bridge is detected
    unsigned long currentMillisBridge = millis();
  if (currentMillisBridge - previousMillisBridge >= intervalBridge) {
    previousMillisBridge = currentMillisBridge;
    if (ledStateBridge == LOW) {
      ledStateBridge = HIGH;
    } else {
      ledStateBridge = LOW;
    }
    if (RGBledStateBridge == LOW) {
      RGB_color(0,255,0);
      RGBledStateBridge = HIGH;
    } else {
      RGBledStateBridge = LOW;
      RGB_color(0,0,0);
    }
digitalWrite(linkerPinker, ledStateBridge);
digitalWrite(rechterPinker, ledStateBridge);  
}
}


//NewPing library code
void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}


//NewPing library code
void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.  
 distanceCm = cm[0];
 distanceCmBridge = cm[1];
}

void ColorLeftRed() { //robot behaviour when red color is detected by left sensor
  digitalWrite (motorA1,LOW);                    
  digitalWrite (motorB1,LOW);
  analogWrite (motorAspeed, 0);
  analogWrite (motorBspeed, 0);
  delay(500);
delay(250);
digitalWrite(linkerPinker, HIGH);
delay(250);
digitalWrite(linkerPinker, LOW);
delay(250);
digitalWrite(linkerPinker, HIGH);
delay(250);
digitalWrite(linkerPinker, LOW);
delay(250);
digitalWrite(linkerPinker, HIGH);
delay(250);
digitalWrite(linkerPinker, LOW);


  digitalWrite (motorA1,LOW);                    
  digitalWrite (motorB1,HIGH);
  analogWrite (motorAspeed, 85);
  analogWrite (motorBspeed, 85);
  delay(4700); // how long the robot takes to do a 360 degrees turn
  
  digitalWrite (motorA1,LOW);                    
  digitalWrite (motorB1,LOW);
  analogWrite (motorAspeed, 0);
  analogWrite (motorBspeed, 0);
  delay(1000);

//Gently turn back to left in case robot is no longer on the track
if(left_sensor_state == HIGH && middle_sensor_state == HIGH && right_sensor_state == HIGH)
{
digitalWrite (motorA1,LOW);                    
  digitalWrite (motorB1,HIGH);
  analogWrite (motorAspeed, 85);
  analogWrite (motorBspeed, 85);
  delay(20); //
  digitalWrite (motorA1,LOW);                    
  digitalWrite (motorB1,LOW);
  analogWrite (motorAspeed, 0);
  analogWrite (motorBspeed, 0);
delay(10);
}  

colorLeftRightAan = false; //turn off color recognition
colorSensorDelay = millis(); //Timer start
}

void ColorRightGreen() {  //robot behaviour when green color is detected by right sensor
  digitalWrite (motorA1,LOW);                    
  digitalWrite (motorB1,LOW);
  analogWrite (motorAspeed, 0);
  analogWrite (motorBspeed, 0);
  delay(500);
delay(250);
digitalWrite(rechterPinker, HIGH);
delay(250);
digitalWrite(rechterPinker, LOW);
delay(250);
digitalWrite(rechterPinker, HIGH);
delay(250);
digitalWrite(rechterPinker, LOW);
delay(250);
digitalWrite(rechterPinker, HIGH);
delay(250);
digitalWrite(rechterPinker, LOW);


  digitalWrite (motorA1,HIGH);                    
  digitalWrite (motorB1,LOW);
  analogWrite (motorAspeed, 85);
  analogWrite (motorBspeed, 85);
  delay(4700); // how long the robot takes to do a 360 degrees turn
   
  digitalWrite (motorA1,LOW);                    
  digitalWrite (motorB1,LOW);
  analogWrite (motorAspeed, 0);
  analogWrite (motorBspeed, 0);
  delay(1000);

//gently turn to the right in case the robot is no longer on the track
if(left_sensor_state == HIGH && middle_sensor_state == HIGH && right_sensor_state == HIGH)
{
digitalWrite (motorA1,HIGH);                    
  digitalWrite (motorB1,LOW);
  analogWrite (motorAspeed, 85);
  analogWrite (motorBspeed, 85);
  delay(20); //
  digitalWrite (motorA1,LOW);                    
  digitalWrite (motorB1,LOW);
  analogWrite (motorAspeed, 0);
  analogWrite (motorBspeed, 0);
delay(10);
}  

colorLeftRightAan = false; //Turn off color recognition
colorSensorDelay = millis(); //Timer start
}
