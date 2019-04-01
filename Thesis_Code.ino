#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
//********************************************************** Setup  *********************************************************************************************
LSM9DS1 imuA; // LEFT IMU, Addresses: (M)0x1C & (AG)0x6A
LSM9DS1 imuB; // RIGHT IMU, Addresses: (M)0x1E & (AG)0x6B
//Flex Sensor pins to Analogue 0 & 1
const int FLEX_PIN_L = A0;
const int FLEX_PIN_R = A1;
const int EMG = A3;

// Global variables to keep track of update rates
unsigned long startTime;
unsigned int accelReadCounter = 0;
unsigned int gyroReadCounter = 0;
unsigned int magReadCounter = 0;
unsigned int tempReadCounter = 0;

const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 47500.0; // Measured resistance of 47k resistor
const float STRAIGHT_RESISTANCE = 37300.0; // resistance when straight
const float BEND_RESISTANCE = 90000.0; // resistance at 90 deg
int i = 0; // Counter Variables
int j = 0;

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
#define PRINT_SPEED 1000 // 1s between prints
static unsigned long lastPrintA = 0; // Keep track of print time
static unsigned long lastPrintB = 0; // Keep track of print time

////CONFIGURE IMU/////
void setupDevice()
{

  imuA.settings.device.commInterface = IMU_MODE_I2C;
  imuB.settings.device.commInterface = IMU_MODE_I2C;

  imuA.settings.device.agAddress = 0x6B; // I2C address 0x6C
  imuB.settings.device.agAddress = 0x6A; // I2C address 0x6C

}

void setupGyro(){
  imuA.settings.gyro.enabled = true;  // Enable the gyro
  imuB.settings.gyro.enabled = true;  // Enable the gyro
  imuA.settings.gyro.scale = 500; // Set scale to +/-500dps
  imuB.settings.gyro.scale = 500; // Set scale to +/-500dps
  imuA.settings.gyro.sampleRate = 5; // 476Hz ODR
  imuB.settings.gyro.sampleRate = 5; // 476Hz ODR
  imuA.settings.gyro.bandwidth = 3;
  imuB.settings.gyro.bandwidth = 3;
  imuA.settings.gyro.lowPowerEnable = false; // LP mode off
  imuB.settings.gyro.lowPowerEnable = false; // LP mode off
  imuA.settings.gyro.HPFEnable = false; // HPF disabled
  imuB.settings.gyro.HPFEnable = false; // HPF disabled
  imuA.settings.gyro.HPFCutoff = 1; // HPF cutoff = 4Hz
  imuB.settings.gyro.HPFCutoff = 1; // HPF cutoff = 4Hz
//imuA
  imuA.settings.gyro.flipX = true; // Don't flip X
  imuA.settings.gyro.flipY = false; // Don't flip Y
  imuA.settings.gyro.flipZ = false; // Don't flip Z
//imuB
  imuB.settings.gyro.flipX = true; // Don't flip X
  imuB.settings.gyro.flipY = false; // Don't flip Y
  imuB.settings.gyro.flipZ = false; // Don't flip Z
}

void setupAccel(){
  imuA.settings.accel.enabled = true; // Enable accelerometer
  imuB.settings.accel.enabled = true; // Enable accelerometer
  //imuA
  imuA.settings.accel.enableX = true; // Enable X
  imuA.settings.accel.enableY = true; // Enable Y
  imuA.settings.accel.enableZ = true; // Enable Z
  //imuB
  imuB.settings.accel.enableX = true; // Enable X
  imuB.settings.accel.enableY = true; // Enable Y
  imuB.settings.accel.enableZ = true; // Enable Z
  imuA.settings.accel.scale = 8; // Set accel scale to +/-8g.
  imuB.settings.accel.scale = 8; // Set accel scale to +/-8g.
  imuA.settings.accel.sampleRate = 1; // Set accel to 10Hz.
  imuB.settings.accel.sampleRate = 1; // Set accel to 10Hz.  
  imuA.settings.accel.bandwidth = -1; // BW = determined by sample rate
  imuB.settings.accel.bandwidth = -1; // BW = determined by sample rate
  imuA.settings.accel.highResEnable = true; // Enabled HR
  imuB.settings.accel.highResEnable = true; // Enabled HR
  imuA.settings.accel.highResBandwidth = 0;
  imuB.settings.accel.highResBandwidth = 0;  
}

uint16_t initLSM9DS1()
{
  setupDevice(); // Setup general device parameters
  setupGyro(); // Set up gyroscope parameters
  setupAccel(); // Set up accelerometer parameters  
}

////CONFIGURATION/////
void setup() 
{
  Serial.begin(115200);
  //Establish the Analog pins as inputs
  pinMode(FLEX_PIN_R, INPUT);
  pinMode(FLEX_PIN_L, INPUT);
  pinMode(EMG, INPUT);

  //Error loops incase IMUs are undetectable
 /* if (!imuA.begin())
  {
    Serial.println("Failed to communicate with imuA LSM9DS1.");
    Serial.println("Double-check wiring for the left IMU.");
    while (1)
      ;
  }
    if (!imuB.begin())
  {
    Serial.println("Failed to communicate with imuA LSM9DS1.");
    Serial.println("Double-check wiring for the right IMU.");
    while (1);
  }
  */
}


//********************************************************** Sensor Readings *********************************************************************************************

//***************************** IMU Sensor Readings *****************************

//***************************** Gyro *****************************
//Left Foot (IMU A)
void printGyroL(float gx, float gy, float gz){
  Serial.println("Gyroscope Data:");
  Serial.println("Left Foot: ");
  Serial.print("X:  ");
  Serial.print(imuA.calcGyro(gx), 2);
  Serial.print(", ");
  Serial.print("Y:  ");
  Serial.print(imuA.calcGyro(gy), 2);
  Serial.print(", ");
  Serial.print("Z:  ");
  Serial.println(imuA.calcGyro(gz), 2);
}
//Right Foot (IMU B)
void printGyroR(float gx, float gy, float gz){
  Serial.println("Right Foot: ");
  Serial.print("X:  ");
  Serial.print(imuB.calcGyro(gx), 2);
  Serial.print(", ");
  Serial.print("Y:  ");
  Serial.print(imuB.calcGyro(gy), 2);
  Serial.print(", ");
  Serial.print("Z:  ");
  Serial.println(imuB.calcGyro(gz), 2);
}

//***************************** Accelerometer ********************
//Left Foot (IMU A)
void printAccelL(float ax, float ay, float az){  
  Serial.println("Accelerometer Data:");
  Serial.println("Left Foot: ");
  Serial.print("X:  ");
  Serial.print(imuA.calcAccel(ax), 2);
  Serial.print(", ");
  Serial.print("Y:  ");
  Serial.print(imuA.calcAccel(ay), 2);
  Serial.print(", ");
  Serial.print("Z:  ");
  Serial.println(imuA.calcAccel(az), 2);
}
//Right Foot (IMU B)
void printAccelR(float ax, float ay, float az){  
  Serial.println("Right Foot: ");
  Serial.print("X:  ");
  Serial.print(imuB.calcAccel(ax), 2);
  Serial.print(", ");
  Serial.print("Y:  ");
  Serial.print(imuB.calcAccel(ay), 2);
  Serial.print(", ");
  Serial.print("Z:  ");
  Serial.println(imuB.calcAccel(az), 2);
}


//***************************** Flex Sensor Readings *****************************
//LEFT
void printFlexL()
{
  int Flex_L = analogRead(A0);
  float flexV_L = Flex_L * VCC / 1023.0;
  float flexR_L = R_DIV * (VCC / flexV_L - 1.0);
  float angle_L = map(flexR_L, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 90.0);
  Serial.println("Left Foot Angle:  " + String(angle_L) + " degrees, Raw reading:  " + Flex_L);
}

//RIGHT
void printFlexR()
{
  int Flex_R = analogRead(A1);
  float flexV_R = Flex_R * VCC / 1023.0;
  float flexR_R = R_DIV * (VCC / flexV_R - 1.0);
  float angle_R = map(flexR_R, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 90.0);
  Serial.println("Right Foot Angle:  " + String(angle_R) + " degrees, Raw reading:  " + Flex_R);
}


//***************************** EMG Readings ************************************
void printEmg()
{
  int emg = analogRead(A2);
  Serial.println("EMG Value: " + String(emg));
}

//********************************************************** Comparisons *********************************************************************************************
//***************************** IMU comparison *****************************
//Stage 1-------------------------------------------------------------
//Left foot-------------------------------------------------------------
void compareLeftImuStage1(float ax, float ay, float az, float gx, float gy, float gz)
{
    Serial.println("***Left foot, IMU Results: ***");    
    //Obtain IMU Variables
    imuA.readAccel();
    imuA.readGyro();
    int LeftAX = imuA.calcAccel(ax); 
    int LeftAY = imuA.calcAccel(ay);
    int LeftAZ = imuA.calcAccel(az);
//X DIRECTION (FORWARD): LOW PRIORITY
      //If value is correct:
    if(LeftAX <= 0.5 && LeftAX >= -0.5){
        Serial.println("Left IMU AX data correct at Stage 1.");
      }
      //If value is too low:
      else if (LeftAX < -0.5){
        Serial.println("Left IMU AX data too low at Stage 1");
        Serial.println("Error Detected:");
        Serial.println("Left foot moved too far forward upon beginning of manoeuvre: Risk of board flipping.");
        Serial.println("Suggested Action:");
        Serial.println("Keep the ball of your left foot centred over the front bolts of the board when popping the ollie.");
        Serial.println(" Try to guide the board to a flat position with your front foot as it rises.");        
        Serial.println();
      }
      //If value is too high:
      //Larger threshold for error as front foot rising faster doesn't compromise manoeuvre
      else if (LeftAX > 0.5){ 
        Serial.println("Left IMU AX data too high at Stage 1");
        Serial.println("Error Detected:");
        Serial.println("Left foot moved too far back upon beginning of manoeuvre: Risk of board flipping.");
        Serial.println("Suggested Action:");
        Serial.print("Keep the ball of your left foot centred over the front bolts of the board when popping the ollie.");
        Serial.println(" Try to guide the board to a flat position with your front foot as it rises.");  
        Serial.println();
      }
//Y DIRECTION (SIDEWAYS): LOW PRIORITY
    if(LeftAY <= 0  && LeftAY >= -1.5){
        Serial.println("Left IMU AY data correct at Stage 1.");
      }
      //If value is too low:
      else if (LeftAY < -1.5){
        Serial.println("Left IMU AY data too low at Stage 1");
        Serial.println("Error Detected:");
        Serial.println("Left foot moved too far away from the front of the board: Risk of 'credit card' injury.");
        Serial.println("Suggested Action:");
        Serial.print("Keep the ball of your left foot centred over the front bolts of the board when popping the ollie.");
        Serial.println(" Try to guide the board to a flat position with your front foot as it rises."); 
        Serial.println();
      }
      //If value is too high:
      //Larger threshold for error as front foot rising faster doesn't compromise manoeuvre
      else if (LeftAY > 0){ 
        Serial.println("Left IMU AY data too high at Stage 1");
        Serial.println("Error Detected:");
        Serial.println("Left foot moved in the wrong direction: Risk of falling.");
        Serial.println("Suggested Action:");
        Serial.print("Keep the ball of your left foot centred over the front bolts of the board when popping the ollie.");
        Serial.println(" Try to guide the board to a flat position with your front foot as it rises."); 
        Serial.println();
      }
//Z DIRECTION (UP): HIGH PRIORITY
      //If value is correct:
    if(LeftAZ <= 0.8 && LeftAZ >= 0.5){
        Serial.println("Left IMU AZ data correct at Stage 1.");
      }
      //If value is too low:
      else if (LeftAZ < 0.5){
        Serial.println("Left IMU AZ data too low at Stage 1");
        Serial.println("Error Detected:");
        Serial.println("Front foot not risen enough upon beginning of manoeuvre: Board will not rise.");
        Serial.println("Suggested Action:");
        Serial.println("Lift your foot up with the front of the board as the tail of the board hits the ground.");
        Serial.println();
      }
      //If value is too high:
      //Larger threshold for error as front foot rising faster doesn't compromise manoeuvre
      else if (LeftAZ > 0.8){ 
        Serial.println("Left IMU AZ data too high at Stage 1");
        Serial.println("Error Detected:");
        Serial.println("Front foot risen too much upon beginning of manoeuvre: Risk of falling.");
        Serial.println("Suggested Action:");
        Serial.println("Keep your foot closer to the board, try to lift your foot along with the board as it rises.");
        Serial.println();
      }
}
//Right foot------------------------------------------------------------
void compareRightImuStage1(float ax, float ay, float az, float gx, float gy, float gz)
{
    Serial.println("***Right foot, IMU Results: ***");    
    //Obtain IMU Variables
    imuB.readAccel();
    imuB.readGyro();
    int RightAX = imuB.calcAccel(ax);
    int RightAY = imuB.calcAccel(ay);
    int RightAZ = imuB.calcAccel(az);
//X DIRECTION (FORWARD): LOW PRIORITY
    if(RightAX <= 0.1  && RightAX >= 0.6){
        Serial.println("Right IMU AX data correct at Stage 1.");
      }
      //If value is too low:
      else if (RightAX < 0.1){
        Serial.println("Right IMU AX data too low at Stage 1");
        Serial.println("Error Detected:");
        Serial.println("Right foot not centred over board: Risk of falling.");
        Serial.println("Suggested Action:");
        Serial.println("Keep your right foot over the centre of the board. Moving your foot to the side will cause the board to spin and increase the risk of injury.");
        Serial.println();
      }
      //If value is too high:
      //Larger threshold for error as front foot rising faster doesn't compromise manoeuvre
      else if (RightAX > 0.6){ 
        Serial.println("Right IMU AX data too high at Stage 1");
        Serial.println("Error Detected:");
        Serial.println("Right foot not centred over board: Risk of falling.");
        Serial.println("Suggested Action:");
        Serial.println("Keep your right foot over the centre of the board. Moving your foot to the side will cause the board to spin and increase the risk of injury.");
        Serial.println();
      }
//Y DIRECTION (SIDEWAYS): LOW PRIORITY
    if(RightAY <= -1  && RightAY >= -.2){
        Serial.println("Right IMU AY data correct at Stage 1.");
      }
      //If value is too low:
      else if (RightAY < -1){
        Serial.println("Right IMU AY data too low at Stage 1");
        Serial.println("Error Detected:");
        Serial.println("Right foot not centred over board: Risk of falling.");
        Serial.println("Suggested Action:");
        Serial.println("Keep your right foot over the centre of the board. Moving your foot to the side will cause the board to spin and increase the risk of injury.");
        Serial.println();      
      }
      //If value is too high:
      //Larger threshold for error as front foot rising faster doesn't compromise manoeuvre
      else if (RightAY > -0.2){ 
        Serial.println("Right IMU AY data too high at Stage 1");
        Serial.println("Error Detected:");
        Serial.println("Right foot not centred over board: Risk of falling.");
        Serial.println("Suggested Action:");
        Serial.println("Keep your right foot over the centre of the board. Moving your foot to the side will cause the board to spin and increase the risk of injury.");
        Serial.println();
      }
//Z DIRECTION (UP): HIGH PRIORITY
      //If value is correct:
    if(RightAZ <= 0.1 && RightAZ >= 1){
        Serial.print("Right IMU AZ data correct at Stage 1.");
      }
      //If value is too low:
      else if (RightAZ < 0.1){
        Serial.println("Right IMU AZ data too low at Stage 1");
        Serial.println("Error Detected:");
        Serial.println("Not enough force exerted on tail of board: Risk of board not rising.");
        Serial.println("Suggested Action:");
        Serial.println("Kick downwards into the tail of the board upon starting the trick. When the board hits the ground, jump with it as it rises.");
        Serial.println();
      }
      //If value is too high:
      //Larger threshold for error as front foot rising faster doesn't compromise manoeuvre
      else if (RightAZ > 1){ 
        Serial.println("Right IMU AZ data too high at Stage 1");
        Serial.println("Error Detected:");
        Serial.println("Back foot risen too much upon beginning of manoeuvre: Risk of board not rising.");
        Serial.println("Suggested Action:");
        Serial.println("Kick downwards into the tail of the board, try to lift your foot along with the board after it pops off the ground.");
        Serial.println();
      }
}

//Stage 2-----------------------------------------------------------------------------------------------------------------------
//Left foot------------------------------------------------------------
void compareLeftImuStage2(float ax, float ay, float az, float gx, float gy, float gz){
    Serial.println("***Left foot, IMU Results: ***");    
    //Obtain IMU Variables
    imuA.readAccel();
    imuA.readGyro();
    int Left2AX = imuA.calcAccel(ax); 
    int Left2AY = imuA.calcAccel(ay);
    int Left2AZ = imuA.calcAccel(az);
    int Left2GX = imuA.calcGyro(gx);
    int Left2GY = imuA.calcGyro(gy);
    int Left2GZ = imuA.calcGyro(gz);

    if(Left2AX <= -2 && Left2AX >= 2){
        Serial.println("Left IMU AX data correct at Stage 2.");
      }
      //If value is too low:
      else if (Left2AX < -2){
        Serial.println("Left IMU AX data too low at Stage 2");
        Serial.println("Error Detected:");
        Serial.println("Right foot moving from above board: Risk of falling.");
        Serial.println("Suggested Action:");
        Serial.println("Keep your foot centred above the board and follow it to ground. Try to land with your foot on the front bolts of the board.");
        Serial.println();
      }
      //If value is too high:
      //Larger threshold for error as front foot rising faster doesn't compromise manoeuvre
      else if (Left2AX > 2){ 
        Serial.println("Left IMU AX data too high at Stage 2.");
        Serial.println("Error Detected:");
        Serial.println("Left foot moving from above board: Risk of falling.");
        Serial.println("Suggested Action:");
        Serial.println("Keep your foot centred above the board and follow it to ground. Try to land with your foot on the front bolts of the board.");
        Serial.println();
      }
//Y DIRECTION (SIDEWAYS): LOW PRIORITY ( RANGE OF -2 : -0.1)
    if(Left2AY <= 0.5   && Left2AY >= -3){
        Serial.println("Left IMU AY data correct at Stage 2.");
      }
      //If value is too low:
      else if (Left2AY < -3){
        Serial.println("Left IMU AY data too low at Stage 2.");
        Serial.println("Error Detected:");
        Serial.println("Left foot moving from above board: Risk of falling.");
        Serial.println("Suggested Action:");
        Serial.println("Keep your foot centred above the board and follow it to ground. Try to land with your foot on the front bolts of the board.");
        Serial.println();
      }
      //If value is too high:
      else if (Left2AY > 0.5){ 
        Serial.println("Left IMU AY data too high at Stage 2.");
        Serial.println("Error Detected:");
        Serial.println("Left foot moving from above board: Risk of falling.");
        Serial.println("Suggested Action:");
        Serial.println("Keep your foot centred above the board and follow it to ground. Try to land with your foot on the front bolts of the board.");
        Serial.println();
      }
//Z DIRECTION (UP): HIGH PRIORITY
      //If value is correct:
    if(Left2AZ <= -2 && Left2AZ >= 2){
        Serial.println("Left IMU AZ data correct at Stage 2.");
      }
      //If value is too low:
      else if (Left2AZ < 2){
        Serial.println("Left IMU AZ data too low at Stage 2");
        Serial.println("Error Detected:");
        Serial.println("Left foot following the board to ground to closely: Risk of board rotation.");
        Serial.println("Suggested Action:");
        Serial.println("Upon reaching stage 2 of the trick, guide the bord downwards to the ground. Do not exert too much force, as the board may spin or rotate.");
        Serial.println();
      }
      //If value is too high:
      //Larger threshold for error as front foot rising faster doesn't compromise manoeuvre
      else if (Left2AZ > 2){ 
        Serial.println("Left IMU AZ data too high at Stage 2");
        Serial.println("Error Detected:");
        Serial.println("Left foot not following the board to ground close enough: Risk of falling.");
        Serial.println("Suggested Action:");
        Serial.println("Upon reaching stage 2 of the trick, follow the board to the ground with your feet to prevent falling.");
        Serial.println();
      }
}
        
//Right foot------------------------------------------------------------
void compareRightImuStage2(float ax, float ay, float az, float gx, float gy, float gz){
    Serial.println("***Right foot, IMU Results: ***");    
    //Obtain IMU Variables
    imuB.readAccel();
    imuB.readGyro();
    int Right2AX = imuB.calcAccel(ax);
    int Right2AY = imuB.calcAccel(ay);
    int Right2AZ = imuB.calcAccel(az);
    int Right2GX = imuB.calcGyro(gx);
    int Right2GY = imuB.calcGyro(gy);
    int Right2GZ = imuB.calcGyro(gz);
//X DIRECTION (FORWARD): LOW PRIORITY
    if(Right2AX <= -0.5 && Right2AX >= 1.1){
        Serial.println("Right IMU AX data correct at Stage 2.");
      }
      //If value is too low:
      else if (Right2AX < -0.5){
        Serial.print("Right IMU AX data too low at Stage 2");
        Serial.println("Error Detected:");
        Serial.println("Right foot moving from above board: Risk of falling.");
        Serial.println("Suggested Action:");
        Serial.println("Keep your foot centred above the board and follow it to ground. Try to land with your foot on the back bolts of the board.");
        Serial.println();
      }
      //If value is too high:
      //Larger threshold for error as front foot rising faster doesn't compromise manoeuvre
      else if (Right2AX > 1.1){ 
        Serial.println("Right IMU AX data too high at Stage 2");
        Serial.println("Error Detected:");
        Serial.println("Right foot moving from above board: Risk of falling.");
        Serial.println("Suggested Action:");
        Serial.println("Keep your foot centred above the board and follow it to ground. Try to land with your foot on the back bolts of the board.");
        Serial.println();
      }
//Y DIRECTION (SIDEWAYS): LOW PRIORITY
    if(Right2AY <=  -2.5 && Right2AY >= 2.5 ){
        Serial.println("Right IMU AY data correct at Stage 2.");
      }
      //If value is too low:
      else if (Right2AY < -2.5){
        Serial.println("Right IMU AY data incorrect at Stage 2");
        Serial.println("Error Detected:");
        Serial.println("Right foot moving from above board: Risk of falling.");
        Serial.println("Suggested Action:");
        Serial.println("Keep your foot centred above the board and follow it to ground. Try to land with your foot on the bolts of the board.");
        Serial.println();
      }
      //If value is too high:
      else if (Right2AY > 2.5){ 
        Serial.println("Right IMU AY data incorrect at Stage 2");
        Serial.println("Error Detected:");
        Serial.println("Right foot moving from above board: Risk of falling.");
        Serial.println("Suggested Action:");
        Serial.println("Keep your foot centred above the board and follow it to ground. Try to land with your foot on the bolts of the board.");
        Serial.println();
      }
//Z DIRECTION (UP): HIGH PRIORITY
      //If value is correct:
    if(Right2AZ <= -2.2 && Right2AZ >= 2.2){
        Serial.print("Right IMU AZ data correct at Stage 2.");
      }
      //If value is too low:
      else if (Right2AZ < -2.2){
        Serial.println("Right IMU AZ data too low at Stage 2");
        Serial.println("Error Detected:");
        Serial.println("Right foot following the board to ground to closely: Risk of board rotation.");
        Serial.println("Suggested Action:");
        Serial.println("Upon reaching stage 2 of the trick, guide the bord downwards to the ground. Do not exert too much force, as the board may spin or rotate.");
        Serial.println();
      }
      //If value is too high:
      //Larger threshold for error as front foot rising faster doesn't compromise manoeuvre
      else if (Right2AZ > 2.2){ 
        Serial.println("Right IMU AZ data too high at Stage 2");
        Serial.println("Error Detected:");
        Serial.println("Right foot not following the board to ground: Risk of falling.");
        Serial.println("Suggested Action:");
        Serial.println("Upon reaching stage 2 of the trick, follow the board to the ground with your feet to prevent falling.");
        Serial.println();
      }
}


//***************************** FLEX COMPARISON *****************************
void compareFlexStage1(){
    //Left Foot
    Serial.println("***Flex Sensor Results: ***");    
    int Flex_L = analogRead(A0);    
    int Flex_R = analogRead(A1);
    if(Flex_L <= 525   && Flex_L >= 350  ){
      Serial.println("Left Foot Flex data correct at Stage 1");
    }
    else if (Flex_L < 390){
      Serial.println("Left Foot Flex data too low at Stage 1.");
      Serial.println("Error Detected:");      
      Serial.println("Ankle not angled enough to compress and pop the board.");
      Serial.println("Suggested Action:");
      Serial.print("Crouch (compress) down over the board before attempting the manoeuvre. ");
      Serial.println("Crouching bulds potential energy, which can be used to 'pop' the board.");
      Serial.println();
    }
     else if (Flex_L > 525){
      Serial.println("Left Foot Flex data too high at Stage 1.");
      Serial.println("Error Detected:");
      Serial.println("Ankle angled too much to get away from the board.");
      Serial.println("Suggested Action:");
      Serial.print("Crouch (compress) down slightly over the board, but maintain a standing position. ");
      Serial.println("Crouching bulds potential energy, which can be used to 'pop' the board, ");
      Serial.println(" however crouching too much over the board prevents it from rising upon 'popping'.");
      Serial.println();
    }  
    //Right foot
    if(Flex_R <= 450  && Flex_R >= 375 ){
      Serial.println("Right Foot Flex data correct at Stage 1");
    }
    else if (Flex_R < 375){
      Serial.println("Right Foot Flex data too low at Stage 1.");
      Serial.println("Error Detected:");
      Serial.println("Ankle not angled enough to compress and pop the board.");
      Serial.println("Suggested Action:");
      Serial.print("Crouch (compress) down over the board before attempting the manoeuvre. ");
      Serial.println("Crouching bulds potential energy, which can be used to 'pop' the board.");
      Serial.println();
    }
     else if (Flex_R > 450){
      Serial.println("Right Foot Flex data too high at Stage 1.");
      Serial.println("Error Detected:");
      Serial.println("Ankle angled too much to get away from the board.");
      Serial.println("Suggested Action:");
      Serial.print("Crouch (compress) down slightly over the board, but maintain a standing position. ");
      Serial.println("Crouching bulds potential energy, which can be used to 'pop' the board, ");
      Serial.println(" however crouching too much over the board prevents it from rising upon 'popping'.");
      Serial.println();
    }
}    
void compareFlexStage2(){
      //Left Foot
    Serial.println("***Flex Sensor Results: ***");    
    int Flex_2L = analogRead(A0);    
    int Flex_2R = analogRead(A1);
    if(Flex_2L <= 625   && Flex_2L >= 550 ){
      Serial.println("Left Foot Flex data correct at Stage 2");
    }
    else if (Flex_2L < 550){
      Serial.println("Left Foot Flex data too low at Stage 2.");
      Serial.println("Error Detected:");
      Serial.println("Ankle not angled enough for landing: Risk of injury.");
      Serial.println("Suggested Action:");
      Serial.println("Keep your feet flat to the board so as to prevent slipping or falling off upon landing.");
      Serial.println();
    }
     else if (Flex_2L > 625){
      Serial.println("Left Foot Flex data too high at Stage 2.");
      Serial.println("Error Detected:");
      Serial.println("Ankle angled too much for safe landing: Risk of falling.");
      Serial.println("Suggested Action:");
      Serial.println("Keep your feet flat to the board so as to prevent slipping or falling off upon landing.");
      Serial.println();
    }  
    //Right foot
    if(Flex_2R <= 550  && Flex_2R >= 600  ){
      Serial.println("Right Foot Flex data correct at Stage 2");
    }
    else if (Flex_2R < 500 ){
      Serial.println("Right Foot Flex data too low at Stage 2.");
      Serial.println("Error Detected:");
      Serial.println("Ankle not angled enough for landing: Risk of injury.");
      Serial.println("Suggested Action:");
      Serial.println("Keep your feet flat to the board so as to prevent slipping or falling off upon landing.");
      Serial.println();
    }
     else if (Flex_2R > 600 ){
      Serial.println("Right Foot Flex data too high at Stage 2.");
      Serial.println("Error Detected:");
      Serial.println("Ankle angled too much for safe landing: Risk of falling.");
      Serial.println("Suggested Action:");
      Serial.println("Keep your feet flat to the board so as to prevent slipping or falling off upon landing.");
      Serial.println();
    }
}


//***************************** EMG COMPARISON *****************************

//EMG COMPARISON
//Stage 1 ------------
void compareEmgStage1(){
    Serial.println("***EMG Results: ***");    
    int EMG = analogRead(A2);
    if(EMG >= 500){
      Serial.println("EMG data correct at Stage 1");
    }
    else if (EMG < 500 ){
      Serial.println("EMG data too low at Stage 1.");
      Serial.println("Error Detected:");
      Serial.println("Not enough force exerted to pop board: Board will not rise.");
      Serial.println("Suggested Action:");
      Serial.println("Crouch to build potential energy, which can be used to 'pop' the board.");
      Serial.println();
    }
}

//Stage 2 ------------
void compareEmgStage2(){
    Serial.println("***EMG Results: ***");    
    int EMG = analogRead(A2);
    if(EMG <= 500){
      Serial.println("EMG data correct at Stage 2");
    }
    else if (EMG > 500 ){
      Serial.println("EMG data too low at Stage 2.");
      Serial.println("Error Detected:");
      Serial.println("Too much force exerted on the board after initial pop: Board will not rise.");
      Serial.println("Suggested Action:");
      Serial.println("After exerting force on the tail of the board, jump from the board to allow it to rise underneath you. Failing to jump will prevent the board from rising.");
      Serial.println();
    }
    }
//********************************************************** Main loop *********************************************************************************************
void loop(){
  //imuA Begin//
  // Update the sensor values whenever new data is available
  if ( imuA.gyroAvailable() ){
    imuA.readGyro();
  }
  if ( imuA.accelAvailable() ){
    imuA.readAccel();
  }
  while (j < 1){
  //Countdown to maneuvre - run once
    Serial.println("Prepare to attempt the maneuvre");
    delay(1000);
    Serial.println("Attempt the maneuvre in 5 seconds");
    delay(1000);
    Serial.println("5");
    delay(1000);
    Serial.println("4");
    delay(1000);
    Serial.println("3");
    delay(1000);
    Serial.println("2");
    delay(1000);
    Serial.println("1");  
    delay(1000);
    Serial.println("Go!");
    Serial.println();
    Serial.println();
    j++;
  }
  
  //Gather maneuvre data
    while( i < 2 ){
      //Read intervals at two specific time points - ~15ms & ~35ms
      if (i == 0){
      delay(15);
      Serial.println("******************************  Stage 1: ******************************* "); 
      //printAccelL(imuA.ax, imuA.ay, imuA.az); // Print "A: ax, ay, az"  
      //printAccelR(imuB.ax, imuB.ay, imuB.az); // Print "A: ax, ay, az"  
      //printGyroL(imuA.gx, imuA.gy, imuA.gz);  // Print "G: gx, gy, gz"
      //printGyroR(imuB.gx, imuB.gy, imuB.gz);  // Print "G: gx, gy, gz"
      //printFlexL();  
      //printFlexR();
      //printEmg();
      Serial.println();      
      compareLeftImuStage1(imuA.ax, imuA.ay, imuA.az,  imuA.gx, imuA.gy, imuA.gz);
      compareRightImuStage1(imuB.ax, imuB.ay, imuB.az,  imuB.gx, imuB.gy, imuB.gz);
      compareFlexStage1();
      compareEmgStage1();
      Serial.println();
      Serial.println();
      i++;
      }

      //Stage 2 - Readings taken at 25ms
      if (i == 1){
      delay(20);
      Serial.println("******************************  Stage 2: ******************************  ");
      //printAccelL(imuA.ax, imuA.ay, imuA.az); // Print "A: ax, ay, az"  
      //printAccelR(imuB.ax, imuB.ay, imuB.az); // Print "A: ax, ay, az"  
      //printGyroL(imuA.gx, imuA.gy, imuA.gz);  // Print "G: gx, gy, gz"
      //printGyroR(imuB.gx, imuB.gy, imuB.gz);  // Print "G: gx, gy, gz"
      //printFlexL();  
      //printFlexR();  
      //printEmg();
      Serial.println();
      compareLeftImuStage2(imuA.ax, imuA.ay, imuA.az,  imuA.gx, imuA.gy, imuA.gz);
      compareRightImuStage2(imuB.ax, imuB.ay, imuB.az,  imuB.gx, imuB.gy, imuB.gz);
      compareFlexStage2();
      compareEmgStage2();
      Serial.println();
      Serial.println();
      i++;
      delay(20);
      }
    }
}
