// Included libraries
#include <OneWire.h>
#include <Adafruit_BNO08x.h>
#include <DallasTemperature.h>
#include <PID_v1_bc.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include "SdFat.h"

#define NUM_PIXELS 1 // Status LED

// Define drive motor pins
#define LEFT_PWM_1 9
#define LEFT_PWM_2 10
#define RIGHT_PWM_1 12
#define RIGHT_PWM_2 11

// Define the PWM pins for the stir bar motors
#define BRAK_STIR_PWM_1 A3
#define BRAK_STIR_PWM_2 24
#define PROP_STIR_PWM_1 6
#define PROP_STIR_PWM_2 5

// Define servo pins
#define BRAK_SERVO_PWM 13
#define PROP_SERVO_PWM 4

#define BRAK_TEMP_SENS A1 // Pin for the teperature sensor data line

// Define chip select pin for SD card
#define SD_CS_PIN 23

// Struct for Euler Angles
struct euler_t
{
  float yaw;
  float pitch;
  float roll;
} ypr;

// Create servo objects
Servo brak_servo;
Servo prop_servo;

// Create BNO085 instance
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensor_value;

OneWire one_wire(BRAK_TEMP_SENS);          // Create a OneWire instance to communicate with the sensor
DallasTemperature temp_sensors(&one_wire); // Pass OneWire reference to Dallas Temperature sensor

Adafruit_NeoPixel pixel(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800); // Status LED

// Define files
SdFat sd;
File32 root;
File32 next_file;
File32 data_file;
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);
String file_name;

bool is_file_new = true; // Checks for new file

// The target angle to keep car straight
double goal_angle = 0.0;

// Define IMU variables
double z_angle;          // z-axis angle
double init_angle = 0.0; // initial angle
double angle_diff = 0.0; // z-axis difference

// Temperature threshold
float temp_diff;

// Temperature change
float temp_change;

// Initialize run count for SD card file
int run_count;

// variables to store temperature
double temperature_c; // Current temperature
double init_temp;     // Initial temperature for differential calculation

// KALMAN FILTER variables
double x_temp; // Filtered temperature
double p_temp; // Initial error covariance
// double x_MPU;  // Filtered temperature
// double p_MPU;  // Initial error covariance

// Process noise and measurement noise
double q_temp; // Process noise covariance
double r_temp; // Measurement noise covariance
// double q_MPU;  // Process noise covariance
// double r_MPU;  // Measurement noise covariance

// Keeping track of time
float curr_time = 0;
unsigned long start_time;
bool first_run = true;

int data_size = 2;      // Number of items to log
double data[data_size]; // Data array

// PID Loop variables
double pid_output; // The output correction from the PID algorithm

// The following numbers need to be adjusted through testing
double k_p = 1.5;  // Proportional weighting
double k_i = 0.03; // Integral weighting
double k_d = 0.3;  // Derivative weighting

// Offsets & speeds for left and right wheel
int left_offset = 0;
int right_offset = 0;
int drive_speed = 128;
int max_offset;

// PID control object; input, output, and goal angle are passed by pointer.
PID car_pid(&x_MPU, &pid_output, &goal_angle, k_p, k_i, k_d, DIRECT);

void drive_forward(int speed) // Drive function
{
  // Left wheel
  digitalWrite(LEFT_PWM_1, HIGH);
  analogWrite(LEFT_PWM_2, speed - right_offset);

  // Right wheel
  digitalWrite(RIGHT_PWM_2, HIGH);
  analogWrite(RIGHT_PWM_1, speed - left_offset);
}

void stop_driving() // Stop function
{
  // Left wheel
  digitalWrite(LEFT_PWM_1, HIGH);
  analogWrite(LEFT_PWM_2, 255);

  // Right wheel
  digitalWrite(RIGHT_PWM_2, HIGH);
  analogWrite(RIGHT_PWM_1, 255);
}

void servo_dump(Servo servo, int angle_us, int delay_ms) // Dump reactants into vessel with servo
{
  servo.writeMicroseconds(angle_us); // Rotate to specified position without delay
  delay(delay_ms);                   // Wait specified delay
  servo.writeMicroseconds(500);      // Return to default position
}

void start_stir(int stir_pin_1, int stir_pin_2, int speed) // Start stirring mechanism
{
  digitalWrite(stir_pin_1, LOW);  // For fast decay
  analogWrite(stir_pin_2, 255);   // 100% to overcome stall
  delay(1000);                    // Wait 1 s to spin up
  analogWrite(stir_pin_2, speed); // Set motor to speed obtained through testing
}

void PID_loop() // Update motor speeds according to PID algorithm
{
  car_pid.Compute(); // Run compute algorithm and updates pid_output

  if (pid_output < 0)
  {
    left_offset = abs(round(pid_output)); // If output needs to be adjusted in positive dir (to the right), increase left wheel speed
    right_offset = round(pid_output);     // and decrease right wheel speed.
  }
  else if (pid_output > 0)
  {
    right_offset = abs(round(pid_output)); // If output needs to be adjusted in negative dir (to the left), increase right wheel speed
    left_offset = -round(pid_output);      // and decrease left wheel speed.
  }
  else // Otherwise set both speed offsets to zero if output does not need adjustment.
  {
    left_offset = 0;
    right_offset = 0;
  }
}

void kalman_filter(double x_k, double p_k, double q, double r, double input, bool tempTrue) // Kalman filtering algorithm
{
  // Kalman filter prediction
  double x_k_minus = x_k;     // Predicted next state estimate
  double p_k_minus = p_k + q; // Predicted error covariance for the next state

  // Kalman filter update

  /* Kalman gain: calculated based on the predicted error covariance
  and the measurement noise covariance, used to update the
  state estimate (x_k) and error covariance (p_k) */
  double k = p_k_minus / (p_k_minus + r); // Kalman gain

  // Comparison with actual sensor reading
  x_k = x_k_minus + k * (input - x_k_minus); // Updated state estimate
  p_k = (1 - k) * p_k_minus;                 // Updated error covariance

  if (tempTrue) // Update state for temperature sensor or IMU accordingly
  {
    x_temp = x_k;
    p_temp = p_k;
  }
  else
  {
    // x_MPU = x_k;
    // p_MPU = p_k;
  }
}

void printer(bool serial_true, unsigned long millis_time, double outputs[data_size]) // Output function
{
  if (serial_true) // Print data to serial or SD card file accordingly in .csv format
  {
    Serial.print(millis_time);

    for (int i = 0; i < data_size; i++)
    {
      Serial.print(",");
      Serial.print(outputs[i]);
    }

    Serial.println("");
  }
  else
  {
    data_file.print(millis_time);

    for (int i = 0; i < data_size; i++)
    {
      data_file.print(",");
      data_file.print(outputs[i]);
    }

    data_file.println("");
  }
}

// Enable reports for IMU
void setReports(void)
{
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV))
  {
    Serial.println("Could not enable rotation vector");
  }
}

// Convert Quaternion to Euler Angles to get yaw
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees = false)
{

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees)
  {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

// Pointer function
void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector, euler_t *ypr, bool degrees = false)
{
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void setup() // Setup (executes once)
{
  // Indicate status to be initialized
  pixel.begin();
  pixel.setBrightness(255);
  pixel.show();
  pixel.setPixelColor(0, 255, 0, 0);
  pixel.show();

  // Get time at start
  start_time = millis();

  while (!sd.begin(config))
  {
    delay(1000); // Wait for a second before retrying
  }

  root = sd.open("/", FILE_READ); // Open SD root directory
  run_count = 0;

  while (true)
  {
    next_file = root.openNextFile();

    if (next_file)
    {
      run_count++; // Increment with each existing file
    }
    else
    {
      next_file.close();
      break;
    }
  }

  root.close();

  // Setting to drive motors output mode
  pinMode(LEFT_PWM_1, OUTPUT);
  pinMode(LEFT_PWM_2, OUTPUT);
  pinMode(RIGHT_PWM_1, OUTPUT);
  pinMode(RIGHT_PWM_2, OUTPUT);

  stop_driving(); // Stop driving motors from any residual bootloader code

  // Initialize the stir motor pins as outputs
  pinMode(BRAK_STIR_PWM_1, OUTPUT);
  pinMode(BRAK_STIR_PWM_2, OUTPUT);
  pinMode(PROP_STIR_PWM_1, OUTPUT);
  pinMode(PROP_STIR_PWM_2, OUTPUT);

  // Setting the stir speed
  start_stir(BRAK_STIR_PWM_1, BRAK_STIR_PWM_2, 146);
  start_stir(PROP_STIR_PWM_1, PROP_STIR_PWM_2, 146);

  temp_sensors.begin();                        // Initialize the DS18B20 sensor
  temp_sensors.requestTemperatures();          // Request temperature from all devices on the bus
  init_temp = temp_sensors.getTempCByIndex(0); // Get temperature in Celsius

  bno08x.begin_I2C();
  setReports();

  // Poll IMU a few times & wait for initialization value to stabilize
  for (int i = 0; i < 5; i++)
  {
    bno08x.getSensorEvent(&sensor_value);
    quaternionToEulerRV(&sensor_value.un.arvrStabilizedRV, &ypr, true);

    init_angle = ypr.yaw;
    z_angle = ypr.yaw;
    angle_diff = z_angle - init_angle;

    delay(200);
  }

  // Initialize Kalman filter parameters
  x_temp = init_temp; // Initial state estimate
  p_temp = 0.1;       // Initial error covariance
  q_temp = 0.01;      // Process noise covariance
  r_temp = 0.5;       // Measurement noise covariance
  // x_MPU = z_angle;    // Initial state estimate
  // p_MPU = 1.0;       // Initial error covariance
  // q_MPU = 0.01;      // Process noise covariance
  // r_MPU = 0.1;       // Measurement noise covariance

  // Initialize servo to default position
  brak_servo.attach(BRAK_SERVO_PWM, 500, 2500);
  prop_servo.attach(PROP_SERVO_PWM, 500, 2500);

  // Dump reactants before starting drive
  servo_dump(prop_servo, 2500, 1000);
  servo_dump(brak_servo, 1500, 6000);

  // Set max speed offset according to drive speed
  if (drive_speed < 128)
  {
    max_offset = drive_speed;
  }
  else
  {
    max_offset = 255 - drive_speed;
  }

  // Activate PID
  car_pid.SetMode(AUTOMATIC);

  // The pid outputs between -51 to 51 depending on how the motors should be adjusted. An output of 0 means no change. (This should be adjusted through testing).
  car_pid.SetOutputLimits(-max_offset, max_offset);

  pixel.setPixelColor(0, 0, 0, 255); // Indicate setup complete status
  pixel.show();

  // Start drive motors at full power to overcome stall
  drive_forward(0);
}

void loop() // Loop (main loop)
{
  // Get time on each measurement of sensor
  if (first_run)
  {
    start_time = millis(); // First measurement saved seperately
    curr_time = 0;         // Set current time to zero from start time
    first_run = false;
  }
  else
  {
    curr_time = (millis() - start_time) / 1000; // Taken to check time against first measurement
  }

  temp_sensors.requestTemperatures();              // Request temperature from all devices on the bus
  temperature_c = temp_sensors.getTempCByIndex(0); // Get temperature in Celsius

  bno08x.getSensorEvent(&sensor_value);
  quaternionToEulerRV(&sensor_value.un.arvrStabilizedRV, &ypr, true);

  z_angle = ypr.yaw;
  angle_diff = z_angle - init_angle;

  // Update kalman filters
  kalman_filter(x_temp, p_temp, q_temp, r_temp, temperature_c, true);
  // kalman_filter(x_MPU, p_MPU, q_MPU, r_MPU, z_angle, false);

  // Open csv file
  file_name = "Run_" + String(run_count) + ".csv";
  data_file = sd.open(file_name, FILE_WRITE);

  if (data_file)
  {
    // Writes header if it's a new file
    if (is_file_new)
    {
      data_file.println("Time,Temperature,Filtered Temperature");
      is_file_new = false;
    }

    // Update data array
    data[0] = temperature_c;
    data[1] = x_temp;
    data[2] = angle_diff;
    // data[3] = x_MPU;

    // Write variable data to the file in CSV format
    printer(false, curr_time, data);

    data_file.close();
  }

  temp_diff = -0.068 * curr_time + 1.4; // Update temperature differential
  temp_change = x_temp - init_temp;     // Calculate temperature change

  drive_forward(128); // 50% speed in slow decay mode

  // // Update PID model
  PID_loop();

  if (temp_change >= temp_diff)
  {
    // Stop driving
    stop_driving();

    // Indicate status to be finished
    pixel.setPixelColor(0, 0, 0, 255);
    pixel.show();

    while (1)
      ; // Do nothing for remainder of uptime
  }
}
