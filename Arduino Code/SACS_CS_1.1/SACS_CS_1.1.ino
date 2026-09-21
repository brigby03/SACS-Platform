//Include Libraries
#include "MPU9250.h"

// Thruster Control Pins
const int solenoid1_pin = 2;
const int solenoid2_pin = 3;

// Thruster and Platform Characteristics based on 100 Psi
float maximum_thrust = 1; //in newtons
float minimum_thrust = 0; //in newtons
float nozzle_moment_distance = .243; //perpindicular distance from COM in meters
float maximum_control_torque = 2 * maximum_thrust * nozzle_moment_distance;
float thrust_per_dutyCycle = 1; //in [Newtons per %], this is the conversion factor from desired thrust to actuated dutyCycle

// this variable represents the amount of time spent thrusting during each cycle in milliseconds
int pwm_period = 100; // making pwm_period smaller should increase the performance of the system up to the limit of the solenoid valve

float theta = 10;
float theta_dot = 10;

// Define Gains, values determined from parameter sweep in MATLAB
float Kp = 0.0159;
float Kd = 0.015;

float theta_error = 0, theta_dot_error = 0;
float target_theta = 0, target_theta_dot = 0; // SET THIS VARIABLE FOR DESIRED POSITION AND VELOCITY

float u;
float uCommanded;
float desired_thrust;
float throttle_ratio;
int openGap;
int closeGap;
unsigned long timer;

int schmittNumber = 0;

// Define DeltaOn and DeltaOff values for Schmitt Trigger
float deltaOn = 0.15; //started this value at 0.2, but it would get close to zero and then slow down to the point that it couldn't overcome friction so it wouldn't quite reach zero
float deltaOff= 0.1; //deltaOff MUST BE SMALLER than deltaOn

// Setup for IMU
MPU9250 mpu;




void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) { 
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
  else Serial.println("MPU Connection Successful!");


  // Variable output window to begin:
  Serial.print("nozzle_moment_distance = "); Serial.println(nozzle_moment_distance);
  Serial.print("maximum_control_torque = "); Serial.println(maximum_control_torque);

  // Set pin modes
  pinMode(solenoid1_pin, OUTPUT);
  pinMode(solenoid2_pin, OUTPUT);
  //pinMode(solenoid3_pin, OUTPUT);  // uncomment this when using four-thruster setup
  //pinMode(solenoid4_pin, OUTPUT);  // uncomment this when using four-thruster setup

  Serial.println();
  Serial.println("Starting..."); 
  Serial.println();
  Serial.println();

  //Find initial Schmitt Number for Hysteresis
  mpu.update();
  if(theta*Kp > deltaOff) schmittNumber = 1; //this line assumes that initially, theta_dot = 0;
  else if(theta*Kp < -deltaOff) schmittNumber = -1; //this line assumes that initially, theta_dot = 0;
  else schmittNumber = 0; // If the control logic starts pointing in the correct direction, then it will command nothing since the platform should just stay there
}






void loop() {

  // Read Sensor data and upload to Serial
  mpu.update();
  updateState();

  u = 0; // initialize variable for control torque

  desired_thrust = calculateDesiredThrust(theta, target_theta, theta_dot, target_theta_dot, Kp, Kd, maximum_control_torque); //this thrust is signed


  mpu.update();


  // command the thrusters to actuate ONLY if the schmitt value is not within dead band
  if (schmittNumber == 1)
  {
    if (desired_thrust > deltaOff) fireThrusters(desired_thrust);
    else if(desired_thrust < -deltaOn) fireThrusters(desired_thrust); schmittNumber = -1;
  }
  else if(schmittNumber == -1)
  {
    if (desired_thrust < -deltaOff) fireThrusters(desired_thrust);
    else if(desired_thrust > deltaOn) fireThrusters(desired_thrust); schmittNumber = 1;
  }
  // notice if schmittNumber == 0, the thrusters are never commanded to fire.


  checkCalibrateIMU(); //check for a tare command

}







// functions:


void fireThrusters(float desired_thrust)
{
  throttle_ratio = abs((desired_thrust - minimum_thrust)/thrust_per_dutyCycle); // create x = (y-b)/m for thrust to duty cycle conversion. This represents the ideal case of b = 0, m = 1
  Serial.print("throttle_ratio = "); Serial.println(throttle_ratio);
  openGap = pwm_period * throttle_ratio; // openGap represents the amount of time the valves should stay open in milliseconds
  //Serial.print("open_gap = "); Serial.println(openGap);
  closeGap = pwm_period - openGap; // closeGap represents the amount of time the valves should stay closed in milliseconds
  //Serial.print("close_gap = "); Serial.println(closeGap);
  mpu.update();
  if(desired_thrust>0) // i.e. fire the negative-moment thrusters
  {
    //start by setting the two unused thrusters to LOW
    digitalWrite(solenoid2_pin,LOW);
    //digitalWrite(solenoid4_pin,LOW);  // uncomment this when using four-thruster setup

    //implement pwm control logic for one period of thrusting
    digitalWrite(solenoid1_pin, HIGH);
    //digitalWrite(solenoid3_pin, HIGH);  // uncomment this when using four-thruster setup
    timer = millis();
    while(millis() - timer < openGap)
    {
      mpu.update();
    }


    digitalWrite(solenoid1_pin, LOW);
    //digitalWrite(solenoid3_pin, LOW);  // uncomment this when using four-thruster setup
    timer = millis();
    while(millis() - timer < closeGap)
    {
      mpu.update();
    }
  }
  else if(desired_thrust<0) // i.e. fire the negative-moment thrusters
  {
    //start by setting the two unused thrusters to LOW
    digitalWrite(solenoid1_pin,LOW);
    //digitalWrite(solenoid3_pin,LOW);  // uncomment this when using four-thruster setup

    //implement pwm control logic for one period of thrusting
    digitalWrite(solenoid2_pin, HIGH);
    //digitalWrite(solenoid4_pin, HIGH);  // uncomment this when using four-thruster setup
    timer = millis();
    while(millis() - timer < openGap)
    {
      mpu.update();
    }



    digitalWrite(solenoid2_pin, LOW);
    //digitalWrite(solenoid4_pin, LOW);  // uncomment this when using four-thruster setup
    timer = millis();
    while(millis() - timer < closeGap)
    {
      mpu.update();
    }
  }
}


void checkCalibrateIMU()
{
  // check if the tare command has been sent to the Serial Monitor by Operator
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't')
    {
      Serial.println("Accel Gyro calibration will start in 5sec.");
      Serial.println("Please leave the device still on the flat plane.");
      mpu.verbose(true);
      delay(5000);
      mpu.calibrateAccelGyro();

      Serial.println("Mag calibration will start in 5sec.");
      Serial.println("Please Wave device in a figure eight until done.");
      delay(5000);
      mpu.calibrateMag();

      
      Serial.println("< calibration parameters >");
      Serial.println("accel bias [g]: ");
      Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
      Serial.print(", ");
      Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
      Serial.print(", ");
      Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
      Serial.println();
      Serial.println("gyro bias [deg/s]: ");
      Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
      Serial.print(", ");
      Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
      Serial.print(", ");
      Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
      Serial.println();
      Serial.println("mag bias [mG]: ");
      Serial.print(mpu.getMagBiasX());
      Serial.print(", ");
      Serial.print(mpu.getMagBiasY());
      Serial.print(", ");
      Serial.print(mpu.getMagBiasZ());
      Serial.println();
      Serial.println("mag scale []: ");
      Serial.print(mpu.getMagScaleX());
      Serial.print(", ");
      Serial.print(mpu.getMagScaleY());
      Serial.print(", ");
      Serial.print(mpu.getMagScaleZ());
      Serial.println();


      mpu.verbose(false);
    }
  }
  
}

void calibrateIMU()
{
  // check if the tare command has been sent to the Serial Monitor by Operator
      Serial.println("Accel Gyro calibration will start in 5sec.");
      Serial.println("Please leave the device still on the flat plane.");
      mpu.verbose(true);
      delay(5000);
      mpu.calibrateAccelGyro();

      Serial.println("Mag calibration will start in 5sec.");
      Serial.println("Please Wave device in a figure eight until done.");
      delay(5000);
      mpu.calibrateMag();

      
      Serial.println("< calibration parameters >");
      Serial.println("accel bias [g]: ");
      Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
      Serial.print(", ");
      Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
      Serial.print(", ");
      Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
      Serial.println();
      Serial.println("gyro bias [deg/s]: ");
      Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
      Serial.print(", ");
      Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
      Serial.print(", ");
      Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
      Serial.println();
      Serial.println("mag bias [mG]: ");
      Serial.print(mpu.getMagBiasX());
      Serial.print(", ");
      Serial.print(mpu.getMagBiasY());
      Serial.print(", ");
      Serial.print(mpu.getMagBiasZ());
      Serial.println();
      Serial.println("mag scale []: ");
      Serial.print(mpu.getMagScaleX());
      Serial.print(", ");
      Serial.print(mpu.getMagScaleY());
      Serial.print(", ");
      Serial.print(mpu.getMagScaleZ());
      Serial.println();


      mpu.verbose(false);
}


void updateState()
{
  theta = mpu.getYaw(); Serial.print("                   theta = "); Serial.println(theta);
  theta_dot = mpu.getGyroZ(); Serial.print("theta_dot= "); Serial.println(theta_dot);
}

float calculateDesiredThrust(float theta, float target_theta, float theta_dot, float target_theta_dot, float Kp, float Kd, float maximum_control_torque)
{
  // Calculate Control Input through PD control law
  theta_error = target_theta - theta;
  theta_dot_error = target_theta_dot - theta_dot;
  u = -Kp*theta_error+Kd*theta_dot_error; Serial.println(u);

  mpu.update();

  // contrain the commanded torque to be within the maximum_control_torque
  if(abs(u)>maximum_control_torque){ 
    if(u>0) uCommanded = maximum_control_torque; //change this to .35 if not working
    else uCommanded = -maximum_control_torque;
  } 
  else uCommanded = u;


  desired_thrust = uCommanded/nozzle_moment_distance/2; //equation based on platform setup: divide by 2 since there are 2 nozzles per couple

  return desired_thrust;
}