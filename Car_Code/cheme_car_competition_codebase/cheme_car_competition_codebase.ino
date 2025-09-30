// Included libraries
#include <OneWire.h>
// #include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <DallasTemperature.h>
#include <PID_v1_bc.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

#define NUM_LEDS 1 // Status LED

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
#define LEFT_SERVO_PWM MOSI
#define RIGHT_SERVO_PWM SCK

#define BRAK_TEMP_SENS A1 // Pin for the teperature sensor data line

#define BNO08X_RESET -1 // No reset pin for IMU over I2C, only enabled for SPI

// #define BOOST_I2C 0x75 // This is the address when pin on converter is set to LOW

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
Servo left_servo;
Servo right_servo;

// Create BNO085 instance
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensor_value;

OneWire one_wire(BRAK_TEMP_SENS);          // Create a OneWire instance to communicate with the sensor
DallasTemperature temp_sensors(&one_wire); // Pass OneWire reference to Dallas Temperature sensor

Adafruit_NeoPixel pixel(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800); // Status LED

// The target yaw angle to keep car straight
double goal_yaw = 0.0;

// Define IMU variables
double yaw;            // yaw angle
double init_yaw = 0.0; // initial yaw angle
double yaw_diff = 0.0; // yaw angle difference

// Delta temperature
double temp_diff;

// Temperature change threshold
double temp_change;

// variables to store temperature
double temperature_c; // Current temperature
double init_temp;     // Initial temperature for differential calculation

// KALMAN FILTER variables
double x_temp; // Filtered temperature
double p_temp; // Initial error covariance
double x_IMU;  // Filtered angle
double p_IMU;  // Initial error covariance

// Process noise and measurement noise
double q_temp; // Process noise covariance
double r_temp; // Measurement noise covariance
double q_IMU;  // Process noise covariance
double r_IMU;  // Measurement noise covariance

// Keeping track of time
double curr_time = 0.0f;
unsigned long start_time;

// PID Loop variables
double pid_output; // The output correction from the PID algorithm

// The following numbers need to be adjusted through testing
double k_p = 2.5; // Proportional weighting
double k_i = 3.0; // Integral weighting
double k_d = 1.5; // Derivative weighting

// Offsets & speeds for left and right wheel
int left_offset = 0;
int right_offset = 0;
int servo_angle = 1475;
int max_offset = 1024;

// PID control object; input, output, and goal angle are passed by pointer.
PID car_pid(&yaw_diff, &pid_output, &goal_yaw, k_p, k_i, k_d, DIRECT);

// void init_buck_boost(void)
// {
//   Wire.begin(); // Begin I2C communication

//   // Change internal output voltage to 676.68 mV
//   // Change LSB
//   Wire.beginTransmission(BOOST_I2C);
//   Wire.write(0x00); // Register Address
//   Wire.write(0x5F); // Changed LSB
//   Wire.endTransmission();

//   //  Change MSB
//   Wire.beginTransmission(BOOST_I2C);
//   Wire.write(0x01); // Register Address
//   Wire.write(0x04); // Changed MSB
//   Wire.endTransmission();

//   // Disable current limiter
//   Wire.beginTransmission(BOOST_I2C);
//   Wire.write(0x02); // Register Address
//   Wire.write(0x64); // Changed LSB
//   Wire.endTransmission();

//   // Enable output
//   Wire.beginTransmission(BOOST_I2C);
//   Wire.write(0x06); // Register Address
//   Wire.write(0xA0); // Changed LSB
//   Wire.endTransmission();

//   Wire.end();
// }

void drive_forward(int speed) // Drive function
{
  digitalWrite(LEFT_PWM_1, HIGH);
  digitalWrite(RIGHT_PWM_2, HIGH);
  // analogWrite(LEFT_PWM_2, speed - left_offset);
  // analogWrite(LEFT_PWM_2, speed);
  digitalWrite(LEFT_PWM_2, LOW);
  // analogWrite(RIGHT_PWM_1, speed - right_offset);
  // analogWrite(RIGHT_PWM_1, speed);
  digitalWrite(RIGHT_PWM_1, LOW);
}

void stop_driving(void) // Stop function
{
  digitalWrite(LEFT_PWM_1, HIGH);
  digitalWrite(RIGHT_PWM_2, HIGH);
  // analogWrite(LEFT_PWM_2, 255);
  digitalWrite(LEFT_PWM_2, HIGH);
  // analogWrite(RIGHT_PWM_1, 255);
  digitalWrite(RIGHT_PWM_1, HIGH);
}

void servo_dump(Servo servo, int angle_us, int delay_ms) // Dump reactants into vessel with servo
{
  // Rotate to specified position
  servo.writeMicroseconds(angle_us);
  delay(500);
  servo.writeMicroseconds(angle_us);

  delay(delay_ms); // Wait specified delay

  // Return to default position
  servo.writeMicroseconds(450);
  delay(500);
  servo.writeMicroseconds(450);
}

void start_stir(int stir_pin_1, int stir_pin_2, int speed) // Start stirring mechanism
{
  digitalWrite(stir_pin_1, LOW);  // For fast decay
  analogWrite(stir_pin_2, speed); // Set motor to speed obtained through testing
}

void PID_loop(void) // Update steering angle according to PID algorithm
{
  car_pid.Compute(); // Run compute algorithm and updates pid_output

  int pid_increment = round(pid_output / 114.0) * 114.0;

  if (pid_output != 0)
  {
    left_offset = -round(pid_output);  // If output needs to be adjusted in positive dir (to the right), increase left steer
    right_offset = -round(pid_output); // and decrease right steer.
  }
  else // Otherwise set both steer offsets to zero if output does not need adjustment.
  {
    left_offset = 0;
    right_offset = 0;
  }

  left_servo.writeMicroseconds(servo_angle + left_offset);
  right_servo.writeMicroseconds(servo_angle + right_offset);
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
    x_IMU = x_k;
    p_IMU = p_k;
  }
}

// Enable reports for IMU
void set_reports(void)
{
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR))
  {
    exit(0);
  }
}

// Convert Quaternion to Euler Angles to get yaw
void quaternion_to_euler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees = false)
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
void quaternion_to_euler_RV(sh2_RotationVector_t *rotational_vector, euler_t *ypr, bool degrees = false)
{
  quaternion_to_euler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void setup(void) // Setup (executes once)
{
  // Indicate status to be initialized
  pixel.begin();
  pixel.setBrightness(255);
  pixel.show();
  pixel.setPixelColor(0, 255, 0, 0);
  pixel.show();

  // Get time at start
  start_time = millis();

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

  // init_buck_boost(); // Initialize Buck-Boost Converter

  // Setting the stir speed
  start_stir(BRAK_STIR_PWM_1, BRAK_STIR_PWM_2, 255);
  start_stir(PROP_STIR_PWM_1, PROP_STIR_PWM_2, 255);

  temp_sensors.begin();                        // Initialize the DS18B20 sensor
  temp_sensors.requestTemperatures();          // Request temperature from all devices on the bus
  init_temp = temp_sensors.getTempCByIndex(0); // Get temperature in Celsius

  bno08x.begin_I2C();
  set_reports();

  // Poll IMU a few times & wait for initialization value to stabilize
  for (int i = 0; i < 5; i++)
  {
    bno08x.getSensorEvent(&sensor_value);
    quaternion_to_euler_RV(&sensor_value.un.gameRotationVector, &ypr, true);

    init_yaw = ypr.yaw;
    yaw = ypr.yaw;
    yaw_diff = yaw - init_yaw;

    delay(200);
  }

  // Initialize Kalman filter parameters
  x_temp = init_temp; // Initial state estimate
  p_temp = 0.1;       // Initial error covariance
  q_temp = 0.01;      // Process noise covariance
  r_temp = 0.5;       // Measurement noise covariance
  x_IMU = yaw_diff;   // Initial state estimate
  p_IMU = 0.0;        // Initial error covariance
  q_IMU = 0.01;       // Process noise covariance
  r_IMU = 5.674;      // Measurement noise covariance

  // Initialize servos to default position
  prop_servo.writeMicroseconds(450);
  prop_servo.attach(PROP_SERVO_PWM, 400, 2600);
  prop_servo.writeMicroseconds(450);
  delay(2000);
  brak_servo.writeMicroseconds(450);
  brak_servo.attach(BRAK_SERVO_PWM, 400, 2600);
  brak_servo.writeMicroseconds(450);
  delay(2000);
  left_servo.writeMicroseconds(1475);
  left_servo.attach(LEFT_SERVO_PWM, 400, 2600);
  left_servo.writeMicroseconds(1475);
  delay(2000);
  right_servo.writeMicroseconds(1475);
  right_servo.attach(RIGHT_SERVO_PWM, 400, 2600);
  right_servo.writeMicroseconds(1475);
  delay(2000);

  // Dump reactants before starting drive
  servo_dump(prop_servo, 2500, 3000);
  delay(7000);
  servo_dump(brak_servo, 2500, 3000);

  start_time = micros(); // First measurement saved seperately
  curr_time = 0;         // Set current time to zero from start time

  delay(17000);

  // Activate PID
  car_pid.SetMode(AUTOMATIC);

  // The pid outputs between -1025 and 1025 depending on how steer should be adjusted. An output of 0 means no change.
  car_pid.SetOutputLimits(-max_offset, max_offset);

  pixel.setPixelColor(0, 0, 0, 255); // Indicate setup complete status
  pixel.show();
}

void loop(void) // Loop (main loop)
{
  drive_forward(0); // 100% speed in slow decay mode

  curr_time = (micros() - start_time) / 1000000.0f; // Taken to check time against first measurement

  temp_sensors.requestTemperatures();              // Request temperature from all devices on the bus
  temperature_c = temp_sensors.getTempCByIndex(0); // Get temperature in Celsius

  bno08x.getSensorEvent(&sensor_value);
  quaternion_to_euler_RV(&sensor_value.un.gameRotationVector, &ypr, true);

  yaw = ypr.yaw;
  yaw_diff = yaw - init_yaw;

  // Update kalman filters
  kalman_filter(x_temp, p_temp, q_temp, r_temp, temperature_c, true);
  kalman_filter(x_IMU, p_IMU, q_IMU, r_IMU, yaw_diff, false);

  temp_diff = x_temp - init_temp;
  temp_change = double(0.185) * curr_time - 4.5f; // Calculate temperature change

  // Update PID model
  PID_loop();

  // if (temp_diff <= temp_change)
  // {
  //   // Stop driving
  //   stop_driving();

  //   // Indicate status to be finished
  //   pixel.setPixelColor(0, 0, 255, 0);
  //   pixel.show();

  //   while (1)
  //     ; // Do nothing for remainder of uptime
  // }
}
