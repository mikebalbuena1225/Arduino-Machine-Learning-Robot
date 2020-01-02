
//Michael Balbuena
/*
This is the first half of a robotics project
this program reads sensor data from various components, then sends that data to a slave
which performs actions depending on the data recieved via the I2C Bus.

The arduinos must have there A4 and A5 pins jumped together for the Wire.beginTransmission() to be successful
The other .ino will be on my github shortly along with schematic 
*/

#include <Wire.h> // for master slave coms through I2C

//sensor pin definitions
#define line_tracking 13 //line tracking sensor
#define IR_sensor 12    //object detecting ir sensor
#define temp_humidity 11 //temperature and humidity sensor
#define mic_digital 10 //microphone digital output pin 
#define vibration_sensor 9 //vibration sensor 
#define capacitive_touch 8 //touch sensor
#define light_sensor A0 //analog pin for photoresistor
#define mic_analog A1 //microphone on analog pin A1
#define tilt_sensor 7

const int rgb_R = 7; 
const int rgb_G = 6;
const int rgb_B = 5;

float temperature = 0; //temp from temp_humidity
float humidity = 0;   //humidity from temp_humidity
float light_intensity = 0; //light_intensity from light_sensor
float mic_analog_value = 0; //sound value from microphone 

float captive_touch_value = 0; //touch value from captive touch sensor 
 
bool line_tracking_value = digitalRead(line_tracking);
bool IR_sensor_value = digitalRead(IR_sensor);
bool mic_digital_value = digitalRead(mic_digital);
bool vibration_sensor_value = digitalRead(vibration_sensor);
bool tilt_sensor_value = digitalRead(tilt_sensor);

void setup() {
  Serial.begin(9600);
  sensor_setup();
  sensor_calibration
  (
    temperature, humidity, light_intensity, mic_analog_value,
    captive_touch_value, line_tracking_value, IR_sensor_value,
    mic_digital_value, vibration_sensor_value, tilt_sensor_value
  );
  //starts I2c bus transfer
  Wire.begin(); 
}

void loop() {
  
  Wire.beginTransmission(9); //beginning transmission to device 9
  I2C_bool_transmission();
  I2C_numerical_transmission();
}

void I2C_numerical_transmission()
{
  if (temperature > 75)
    {Wire.write("hot");}
    else if (temperature < 65)
    {Wire.write("cold");}
    else
    { Wire.write("ideal temperature");}

  if (humidity > 60)
    {Wire.write("humid");}
    else if (humidity < 30)
    {Wire.write("dry");}
    else
    {Wire.write("ideal humidity");}

  if(light_intensity > 50)
    {Wire.write("bright");}
    else if (light_intensity < 30)
    {Wire.write("dark");}
    else
    {Wire.write("ideal lighting");}

  if (captive_touch_value > 50)
    {Wire.write("touch");}
    else
    {Wire.write("No touch");}

  if (mic_analog_value > 50)
    {Wire.write("Sensing sound");}
    else
    {Wire.write("No sound sensed");}
}

void I2C_bool_transmission()
{
  if(IR_sensor_value == HIGH)
    {Wire.write(IR_sensor_value);}
  
  if(line_tracking_value == HIGH)
    {Wire.write(line_tracking_value);}
  
  if(tilt_sensor_value == HIGH)
    {Wire.write(tilt_sensor_value);}
  
  if(vibration_sensor_value == HIGH)
    {Wire.write(vibration_sensor_value)}
  
  if(mic_digital_value == HIGH)
    {Wire.write(mic_digital_value);}
}
void sensor_setup()
{
  //slave calibration
  //setting digital sensors as inputs 
  pinMode(line_tracking, INPUT);
  pinMode(IR_sensor, INPUT);
  pinMode(temp_humidity, INPUT);
  pinMode(mic_digital, INPUT);
  pinMode(vibration_sensor, INPUT);
  pinMode(capacitive_touch, INPUT);
  pinMode(tilt_sensor, INPUT);
  //setting analog sensors as inputs
  pinMode(light_sensor, INPUT);
  pinMode(mic_analog, INPUT);
  //setting status output as outputs
  pinMode(rgb_R,OUTPUT);
  pinMode(rgb_G,OUTPUT);
  pinMode(rgb_B,OUTPUT);

  //start up sequence 
  //step 1 sequencing through RGB
  rgb_check();
}

void rgb_check()
{
  digitalWrite(rgb_R, HIGH);
  delay(500);
  digitalWrite(rgb_R, LOW);
  digitalWrite(rgb_G, HIGH);
  delay(500);
  digitalWrite(rgb_G, LOW);
  digitalWrite(rgb_B, HIGH);
  delay(500);
  digitalWrite(rgb_B, LOW);
  digitalWrite(rgb_R, LOW);
  digitalWrite(rgb_G, LOW);
  delay(500);

  digitalWrite(rgb_R, HIGH);
  delay(1000);
  digitalWrite(rgb_R, LOW);
  digitalWrite(rgb_G, HIGH);
  delay(1000);
  digitalWrite(rgb_G, LOW);
  digitalWrite(rgb_B, HIGH);
  delay(1000);
  digitalWrite(rgb_B, LOW);
  digitalWrite(rgb_R, LOW);
  digitalWrite(rgb_G, LOW);
}

void sensor_calibration
(
float temperature, float humidity,
float light_intensity,float mic_analog_value,
float captive_touch_value, 
bool line_tracking_value, bool IR_sensor_value, 
bool mic_digital_value,bool vibration_sensor_value, bool tilt_sensor_value
)
{
  if (line_tracking_value == HIGH)
  {
    Serial.println("On line");
    digitalWrite(rgb_B, HIGH);
    delay(500);
    digitalWrite(rgb_B, LOW);
  }
  else{
    Serial.println("Not on line");
    digitalWrite(rgb_R, HIGH);
    delay(500);
    digitalWrite(rgb_R, LOW);
  }
  if (IR_sensor_value == LOW)
  {
    Serial.println("No edge detected");
    digitalWrite(rgb_B, HIGH);
    delay(500);
    digitalWrite(rgb_B, LOW);
  }
  else
  {
    Serial.println("Edge detected");
    digitalWrite(rgb_R, HIGH);
    delay(500);
    digitalWrite(rgb_R, LOW);
  }
  if (temperature > 75)
  {
    Serial.println("hot");
    digitalWrite(rgb_R, HIGH);
    delay(500);
    digitalWrite(rgb_R, LOW);
  }
  else if (temperature < 65)
  {
    Serial.println("cold");
    digitalWrite(rgb_B, HIGH);
    delay(500);
    digitalWrite(rgb_B, LOW);
  }
  else
  { 
    Serial.println("ideal");
    digitalWrite(rgb_G, HIGH);
    delay(500);
    digitalWrite(rgb_G, LOW);
  }
  if (humidity > 60)
  {
    Serial.println("too humid");
    digitalWrite(rgb_R, HIGH);
    delay(500);
    digitalWrite(rgb_R, LOW);
  }
  else if (humidity < 30)
  {
    Serial.println("too dry");
    digitalWrite(rgb_R, HIGH);
    delay(500);
    digitalWrite(rgb_R, LOW);
  }
  else
  { 
    Serial.println("ideal");
    digitalWrite(rgb_G, HIGH);
    delay(500);
    digitalWrite(rgb_G, LOW);
  }
  if (mic_digital_value == HIGH)
  {
   Serial.println("Too loud");
   digitalWrite(rgb_R, HIGH);
   delay(500);
   digitalWrite(rgb_R, LOW);
  }
  else
  {
    Serial.println("Not too loud");
    digitalWrite(rgb_B, HIGH);
    delay(500);
    digitalWrite(rgb_B, LOW);
  }

  if (vibration_sensor_value == HIGH)
  {
    Serial.println("Sensing vibrations");
    digitalWrite(rgb_B, HIGH);
    delay(500);
    digitalWrite(rgb_B, LOW);
  }
  else
  {
    Serial.println("No vibrations");
    digitalWrite(rgb_R, HIGH);
    delay(500);
    digitalWrite(rgb_R, LOW);
  }
  
  if(tilt_sensor_value == HIGH)
  {
    Serial.println("Sensing tilt");
    digitalWrite(rgb_R, HIGH);
    delay(500);
    digitalWrite(rgb_R, LOW);
  }
  else
  {
   Serial.println("No tilt");
   digitalWrite(rgb_B, HIGH);
   delay(500);
   digitalWrite(rgb_B, LOW);
  }
  
  if (captive_touch_value > 50)
  {
    Serial.println("Sensing touch");
    digitalWrite(rgb_G, HIGH);
    delay(500);
    digitalWrite(rgb_G, LOW);
  }
  else
  {
    Serial.println("No touch sensed");
    digitalWrite(rgb_R, HIGH);
    delay(500);
    digitalWrite(rgb_R, LOW);
  }
  
  if (mic_analog_value > 50)
  {
    Serial.println("Sensing sound");
    digitalWrite(rgb_B, HIGH);
    delay(500);
    digitalWrite(rgb_B, LOW);
  }
  else
  {
    Serial.println("No sound sensed");
    digitalWrite(rgb_R, HIGH);
    delay(500);
    digitalWrite(rgb_R, LOW);
  }
  
  if(light_intensity > 50)
  {
    Serial.println("bright");
    digitalWrite(rgb_R, HIGH);
    delay(500);
    digitalWrite(rgb_R, LOW);
  }
  else if (light_intensity < 30)
  {
    Serial.println("Dark");
    digitalWrite(rgb_B, HIGH);
    delay(500);
    digitalWrite(rgb_B, LOW);
  }
  else
  {
    Serial.println("ideal");
    digitalWrite(rgb_G, HIGH);
    delay(500);
    digitalWrite(rgb_G, LOW);
  }
}
