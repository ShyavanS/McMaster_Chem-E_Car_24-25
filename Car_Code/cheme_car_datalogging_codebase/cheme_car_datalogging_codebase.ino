// Included libraries
#include <OneWire.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <DallasTemperature.h>
#include "DFRobot_EC10.h"
#include <EEPROM.h>
// #include <PID_v1_bc.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include "SdFat.h"

#define NUM_PIXELS 1 // Status LED

// Define drive motor pins
#define right_pwm1 10
#define right_pwm2 9
#define left_pwm1 12
#define left_pwm2 11

// Define the PWM pins for the stir bar motor
#define stirPin1 24
#define stirPin2 A3

// Define servo pin
#define servo_pwm 13

#define ONE_WIRE_BUS A1 // pin for the DS18B20 data line

// Define chip select pin for SD card
#define SD_CS_PIN 23

#define EC_THRESH 100

#define BOOST_I2C 0x75

Servo servo; // Create servo object

MPU6050 mpu(Wire); // Create MPU6050 instance

OneWire oneWire(ONE_WIRE_BUS);       // Create a OneWire instance to communicate with the sensor
DallasTemperature sensors(&oneWire); // Pass oneWire reference to Dallas Temperature sensor

Adafruit_NeoPixel pixel(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800); // Status LED

// Define files
SdFat SD;
File32 root;
File32 nextFile;
File32 dataFile;
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);
String fileName;

bool isFileNew = true; // Checks for new file

// The target angle to keep car straight
double goalAngle = 0.0;

// Define accelerometer variables
double zAngle; // z-axis angle

// Temperature threshold
float tempDiff;

// Initialize run count for SD card file
int runCount;

// Temperature change
float tempChange;

// variables to store temperature
double temperatureC; // Current temperature
double initTemp;     // Initial temperature for differential calculation


double data[2]; // Data array

// KALMAN FILTER variables
  //temp sensor
double r_temp; //measurment noise variance  
double q_temp; //process noise variance -play around later to improve results
double x_k_temp; //initializing estimated status
double p_k_temp; //initializing error covariance
double K_temp; //initializing Kalman gain

// Keeping track of time
float currTime = 0;
unsigned long startTime;
bool firstRun = true;

// PID Loop variables
// double pidOutput; // The output correction from the PID algorithm

// The following numbers need to be adjusted through testing
// double Kp = 1; // Proportional weighting
// double Ki = 0; // Integral weighting
// double Kd = 0; // Derivative weighting

// Offset speeds for left and right wheel
// int left_offset = 0;
// int right_offset = 0;

// PID control object; input, output, and goal angle are passed by pointer.
// PID carPID(&x_MPU, &pidOutput, &goalAngle, Kp, Ki, Kd, DIRECT);

void drive_forward(int speed) // Drive function
{
  // Right wheel
  digitalWrite(right_pwm2, HIGH);
  analogWrite(right_pwm1, speed);
  // analogWrite(right_pwm1, speed - right_offset);

  // Left wheel
  digitalWrite(left_pwm2, HIGH);
  analogWrite(left_pwm1, speed);
  // analogWrite(left_pwm1, speed - left_offset);
}

void stop_driving() // Stop function
{
  // Right wheel
  digitalWrite(right_pwm2, HIGH);
  analogWrite(right_pwm1, 255);

  // Left wheel
  digitalWrite(left_pwm2, HIGH);
  analogWrite(left_pwm1, 255);
}

void servo_dump() // Dump contents of bowl into braking vessel with servo
{
  servo.writeMicroseconds(2600); // Rotate to 180 deg position without delay
  delay(1000);                   // Wait 1 s
  servo.writeMicroseconds(500);  // Return to default position
}


void start_stir() // Start stirring mechanism
{
  analogWrite(stirPin1, 255);  // 100% to overcome stall
  digitalWrite(stirPin2, LOW); // For fast decay
  delay(1000);                 // Wait 1 s to spin up
  analogWrite(stirPin1, 146);  // 57% speed as per testing
}

// void PID_loop() // Update motor speeds according to PID algorithm
// {
//   carPID.Compute(); // Run compute algorithm and updates pidOutput

//   if (pidOutput > 0)
//   {
//     left_offset = abs(round(pidOutput)); // If output needs to be adjusted in positive dir (to the right), increase left wheel speed
//     right_offset = 0;                    // Zero other wheel offset to prevent instability
//   }
//   else if (pidOutput < 0)
//   {
//     right_offset = abs(round(pidOutput)); // If output needs to be adjusted in negative dir (to the left), increase right wheel speed
//     left_offset = 0;                      // Zero other wheel offset to prevent instability
//   }
//   else
//   {
//     left_offset = 0;
//     right_offset = 0;
//   }
// }
double kalman_filter_temperature(double input, double x_k_temp) //void kalman_filter(double x_k, double p_k, double q, double r, double input, bool tempTrue) // Kalman filtering algorithm
{
  double x_k_temp_min1 = x_k_temp;
  double p_k_temp_min1 = p_k_temp;
  K_temp = p_k_temp_min1 / (p_k_temp_min1 + r_temp); //updating Kalman gain
  //original equation is  K = p_K*H / (H*H*p_k+r) but measurment map scalar is 1
  x_k_temp = x_k_temp_min1 + K_temp * (input - x_k_temp_min1); //update state estimate
  p_k_temp = (1 - K_temp) * p_k_temp_min1 + q_temp; //update error covariance
  
  return x_k_temp; //filtered value
}

check_buck_boost_converter()
{
  Wire.beginTransmission(TPS55289_ADDR);
  if(((Wire.read(0x07) >> 7) & 0xFF)){
    buck_boost_converter_short_circuit = 1;
  }
  if(((Wire.read(0x07) >> 6) & 0xFF)){
    buck_boost_converter_overcurrent = 1;
  }
  if(((Wire.read(0x07) >> 5) & 0xFF)){
    buck_boost_converter_overvoltage = 1;
  }
  return 0;
}

void printer(bool serialTrue, unsigned long millisTime, double outputs[2]) // Output function
{
  if (serialTrue) // Print data to serial or SD card file accordingly in .csv format
  {
    Serial.print(millisTime);

    for (int i = 0; i < 2; i++)
    {
      Serial.print(",");
      Serial.print(outputs[i]);
    }

    Serial.println("");
  }
  else
  {
    dataFile.print(millisTime);

    for (int i = 0; i < 2; i++)
    {
      dataFile.print(",");
      dataFile.print(outputs[i]);
    }

    dataFile.println("");
  }
}

void setup() // Setup (executes once)
{
  // Tie pin 4 to GND for compatibility
  pinMode(GND_PIN, OUTPUT);
  digitalWrite(GND_PIN, LOW);

  // Indicate status to be initialized
  pixel.begin();
  pixel.setBrightness(255);
  pixel.show();
  pixel.setPixelColor(0, 255, 0, 0);
  pixel.show();

  // Get time at start
  startTime = millis();

  while (!SD.begin(config))
  {
    delay(1000); // Wait for a second before retrying
  }

  root = SD.open("/", FILE_READ); // Open SD root directory
  runCount = 0;

  while (true)
  {
    nextFile = root.openNextFile();

    if (nextFile)
    {
      runCount++; // Increment with each existing file
    }
    else
    {
      nextFile.close();
      break;
    }
  }

  root.close();

  // Setting to drive motors output mode
  pinMode(right_pwm1, OUTPUT);
  pinMode(right_pwm2, OUTPUT);
  pinMode(left_pwm1, OUTPUT);
  pinMode(left_pwm2, OUTPUT);

  stop_driving(); // Stop driving motors from residual bootloader code

  // Initialize the stir motor pins as outputs
  pinMode(stirPin1, OUTPUT);
  pinMode(stirPin2, OUTPUT);

  // Initialize the stir motor pins as outputs
  pinMode(linAcc1, OUTPUT);
  pinMode(linAcc2, OUTPUT);

  // Setting the stir speed
  start_stir();

  // Inject the contents of the syringe
  inject();

  ec.begin();
  sensors.begin();                       // Initialize the DS18B20 sensor
  sensors.requestTemperatures();         // Request temperature from all devices on the bus
  initTemp = sensors.getTempCByIndex(0); // Get temperature in Celsius

  Wire.begin();             // Initialize I2C communication
  mpu.begin();              // Initialize MPU6050
  mpu.calcOffsets();        // Zero yaw angle
  mpu.update();             // Update MPU readings
  zAngle = mpu.getAngleZ(); // Get z-axis angle from MPU

  // KALMAN FILTER variables
    //temp sensor
  double r_temp = 0.0573; //measurment noise variance  
  double q_temp = 0.01; //process noise variance -play around later to improve results
  double x_k_temp = initTemp; //initializing estimated status
  double p_k_temp = 0; //initializing error covariance
  double K_temp = 0; //initializing Kalman gain

  // Buck boost converter errors
  bool buck_boost_converter_short_circuit = 0;
  bool buck_boost_converter_overcurrent = 0;
  bool buck_boost_converter_overvoltage = 0;

  // Initialize servo to default position
  servo.attach(servo_pwm, 500, 2600);


  // Dump reactants before starting drive
  servo_dump();

  // Activate PID
  // carPID.SetMode(AUTOMATIC);

  // The pid outputs between -51 to 51 depending on how the motors should be adjusted. An output of 0 means no change. (This should be adjusted through testing).
  // carPID.SetOutputLimits(-51, 51);

  pixel.setPixelColor(0, 0, 0, 255); // Indicate setup complete status
  pixel.show();

  // Start drive motors at full power to overcome stall
  drive_forward(0);
}

void loop() // Loop (main loop)
{
  sensors.requestTemperatures();             // Request temperature from all devices on the bus
  temperatureC = sensors.getTempCByIndex(0); // Get temperature in Celsius

  voltage = analogRead(EC_Pin)/1024.0*5000;
  ecValue = ec.readEC(voltage, x_k_temp);

  mpu.update();             // Update MPU readings
  zAngle = mpu.getAngleZ(); // Get z-axis angle from MPU

  // Update kalman filters
  x_k_temp = kalman_filter_temperature(temperatureC, x_k_temp);
  //kalman_filter(x_temp, p_temp, q_temp, r_temp, temperatureC, true);
  //kalman_filter(x_MPU, p_MPU, q_MPU, r_MPU, zAngle, false);

  // Get time on each measurement of sensor
  if (firstRun)
  {
    startTime = millis(); // First measurement saved seperately
    currTime = 0;         // Set current time to zero from start time
    firstRun = false;
  }
  else
  {
    currTime = (millis() - startTime) / 1000; // Taken to check time against first measurement
  }

  // Open csv file
  fileName = "Run_" + String(runCount) + ".csv";
  dataFile = SD.open(fileName, FILE_WRITE);

  if (dataFile)
  {
    // Writes header if it's a new file
    if (isFileNew)
    {
      dataFile.println("Time,Temperature,Filtered Temperature");
      isFileNew = false;
    }

    // Update data array
    data[0] = temperatureC;
    data[1] = x_k_temp;
    //data[2] = zAngle;
    //data[3] = x_MPU;

    // Write variable data to the file in CSV format
    printer(false, currTime, data);

    dataFile.close();
  }

  drive_forward(128); // 50% speed in slow decay mode

  // // Update PID model
  // PID_loop();

  if(ecValue >= EC_THRESH) {
    // Stop driving
    stop_driving();

    // Indicate status to be finished
    pixel.setPixelColor(0, 0, 0, 255);
    pixel.show();

    while (1)
      ; // Do nothing for remainder of uptime
  } 
  if(buck_boost_converter_short_circuit){
    Serial.println("Buck boost converter error - short circuit"); 
  }
  if(buck_boost_converter_overcurrent){
    Serial.println("Buck boost converter error - overcurrent"); 
  }
  if(buck_boost_converter_overvoltage){
    Serial.println("Buck boost converter error - overvoltage"); 
  }
}
