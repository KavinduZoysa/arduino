 
#include <Wire.h>

long gyroXCalli = 0;
long rawOmegaXPresent = 0;
long rawOmegaXPast = 0;
float omegaX = 0.0;
float angelX = 0.0;
float angelXTemp = 0.0;
float angelXFromAccel = 0.0;
long rawAccelX = 0;
float accelX = 0.0;

long rawAccelY = 0;
float accelY = 0.0;

long rawAccelZ = 0;
float accelZ = 0.0;

long timePast = 0;
long timePresent = 0;
float totalAccel = 0.0;

void setup() {
  Serial.begin(57600);
  Wire.begin();
  setUpMPU();
  callibrateGyroValues();
}

void loop() {
  calculateAngel();
  printData();
}

void setUpMPU() {
  // Power management
  Wire.beginTransmission(0b1101000);          // Start the communication by using address of MPU
  Wire.write(0x6B);                           // Access the power management register. 0b01101011
  Wire.write(0b00000000);                     // Set sleep = 0
  Wire.endTransmission();                     // End the communication

  // Configure gyro
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);                           // Access the gyro configuration register
  Wire.write(0b00001000);
  Wire.endTransmission();

  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0b00010000);                                                    //Set the requested starting register
  Wire.endTransmission(); 
}

void callibrateGyroValues() {
    for (int i=0; i<5000; i++) {
      readRawGyroValues();
      gyroXCalli = gyroXCalli + rawOmegaXPresent;
    }
    gyroXCalli = gyroXCalli/5000.0;
}

void calculateAngel() {
  readRawGyroValues();
  rawOmegaXPresent = rawOmegaXPresent - gyroXCalli;                                            
  calculateAngularVelocity();
  
  // angelZ = angelZ + ((timePresentZ - timePastZ)*(gyroZPresent + gyroZPast - 2*gyroZCalli)) / (2*1000*65.5);
  // 1/(1000*2*65.5) = 0.00000763
  // 1000 --> convert milli seconds into seconds
  // 2 --> comes when calculating the area of trapezium
  // substacted the callibated result two times because there are two gyro readings
  timePresent = micros(); 
  long diff = micros() - timePast;
  angelX = angelX + ((timePresent - timePast)*(rawOmegaXPresent + rawOmegaXPast)) * 0.00000000763;
  angelXTemp = angelXTemp + ((timePresent - timePast)*(rawOmegaXPresent + rawOmegaXPast)) * 0.00000000763;
  rawOmegaXPast = rawOmegaXPresent;
  timePast = timePresent;
  calculateAccel(); 
  calculateAngelFromAccel();
  angelX = 0.996*angelX + 0.004*angelXFromAccel;
}

void readRawGyroValues() {
  Wire.beginTransmission(0b1101000);                        
  Wire.write(0x43);                                           
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,2);                     
  while(Wire.available() < 2); // reading only the x-value                                
  rawOmegaXPresent = Wire.read()<<8|Wire.read();                  
}

void calculateAngularVelocity() {
  omegaX = rawOmegaXPresent / 65.5;  
}

void readRawAccelValues() {
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x3B); 
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); 
  while(Wire.available() < 6);
  rawAccelX = Wire.read()<<8|Wire.read(); 
  rawAccelY = Wire.read()<<8|Wire.read(); 
  rawAccelZ = Wire.read()<<8|Wire.read(); 
}

void calculateAccel() {
  readRawAccelValues();
  accelX = rawAccelX/4096.0;
  accelY = rawAccelY/4096.0;
  accelZ = rawAccelZ/4096.0;
  totalAccel = sqrt((accelX*accelX) + (accelY*accelY) + (accelZ*accelZ));
}

void calculateAngelFromAccel() {
  angelXFromAccel = asin((float)accelY/totalAccel)*57.296;
}

void printData() {
  Serial.print(angelX);
  Serial.print("\t");
  Serial.println(angelXTemp);
}
