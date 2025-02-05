// Included libraries
#include <OneWire.h>
// #include <Wire.h>
// #include <MPU6050_light.h>
#include <DallasTemperature.h>
// #include <PID_v1_bc.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

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

// Create servo objects
Servo brak_servo;
Servo prop_servo;

// MPU6050 mpu(Wire); // Create MPU6050 instance

OneWire one_wire(BRAK_TEMP_SENS);          // Create a OneWire instance to communicate with the sensor
DallasTemperature temp_sensors(&one_wire); // Pass OneWire reference to Dallas Temperature sensor

Adafruit_NeoPixel pixel(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800); // Status LED

// The target angle to keep car straight
// double goal_angle = 0.0;

// Define accelerometer variables
// double z_angle; // z-axis angle

// Temperature threshold
float temp_diff;

// Temperature change
float temp_change;

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

// PID Loop variables
// double pid_output; // The output correction from the PID algorithm

// The following numbers need to be adjusted through testing
// double k_p = 1; // Proportional weighting
// double k_i = 0; // Integral weighting
// double k_d = 0; // Derivative weighting

// Offset speeds for left and right wheel
// int left_offset = 0;
// int right_offset = 0;

// PID control object; input, output, and goal angle are passed by pointer.
// PID car_pid(&x_MPU, &pid_output, &goal_angle, k_p, k_i, k_d, DIRECT);

void drive_forward(int speed) // Drive function
{
  // Left wheel
  digitalWrite(LEFT_PWM_1, HIGH);
  analogWrite(LEFT_PWM_2, speed);
  // analogWrite(LEFT_PWM_2, speed - right_offset);

  // Right wheel
  digitalWrite(RIGHT_PWM_2, HIGH);
  analogWrite(RIGHT_PWM_1, speed);
  // analogWrite(RIGHT_PWM_1, speed - left_offset);
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

// void PID_loop() // Update motor speeds according to PID algorithm
// {
//   car_pid.Compute(); // Run compute algorithm and updates pid_output

//   if (pid_output > 0)
//   {
//     left_offset = abs(round(pid_output)); // If output needs to be adjusted in positive dir (to the right), increase left wheel speed
//     right_offset = 0;                    // Zero other wheel offset to prevent instability
//   }
//   else if (pid_output < 0)
//   {
//     right_offset = abs(round(pid_output)); // If output needs to be adjusted in negative dir (to the left), increase right wheel speed
//     left_offset = 0;                      // Zero other wheel offset to prevent instability
//   }
//   else
//   {
//     left_offset = 0;
//     right_offset = 0;
//   }
// }

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

  // Wire.begin();             // Initialize I2C communication
  // mpu.begin();              // Initialize MPU6050
  // mpu.calcOffsets();        // Zero yaw angle
  // mpu.update();             // Update MPU readings
  // z_angle = mpu.getAngleZ(); // Get z-axis angle from MPU

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

  // Cut power to servos
  brak_servo.detach();
  prop_servo.detach();

  // Activate PID
  // car_pid.SetMode(AUTOMATIC);

  // The pid outputs between -51 to 51 depending on how the motors should be adjusted. An output of 0 means no change. (This should be adjusted through testing).
  // car_pid.SetOutputLimits(-51, 51);

  pixel.setPixelColor(0, 0, 0, 255); // Indicate setup complete status
  pixel.show();

  // Start drive motors at full power to overcome stall
  drive_forward(0);
}

void loop() // Loop (main loop)
{
  temp_sensors.requestTemperatures();              // Request temperature from all devices on the bus
  temperature_c = temp_sensors.getTempCByIndex(0); // Get temperature in Celsius

  // mpu.update();             // Update MPU readings
  // z_angle = mpu.getAngleZ(); // Get z-axis angle from MPU

  // Update kalman filters
  kalman_filter(x_temp, p_temp, q_temp, r_temp, temperature_c, true);
  // kalman_filter(x_MPU, p_MPU, q_MPU, r_MPU, z_angle, false);

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

  temp_diff = -0.068 * curr_time + 1.4; // Update temperature differential
  temp_change = x_temp - init_temp;     // Calculate temperature change

  drive_forward(128); // 50% speed in slow decay mode

  // // Update PID model
  // PID_loop();

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
