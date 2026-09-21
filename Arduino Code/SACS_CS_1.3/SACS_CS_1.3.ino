// SACS 1.30
// PWPF Modulation using Schmitt Trigger for 1-Axis stabilization

//Include Libraries
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
#include <IRremote.hpp>

// Pin Definitions
const int solenoid1_pin = 3;
const int solenoid2_pin = 4;
const int remote_pin = 15;
#define IR_RECEIVE_PIN remote_pin

// Declare State Variables
float theta; // [deg]
float theta_dot; // [deg]

// Define Error Variables and Reference Targets
float target_theta = 0, target_theta_dot = 0; // [deg] *SET THIS VARIABLE FOR DESIRED POSITION AND VELOCITY*
float theta_error; float theta_dot_error; // [rad], [rad/s]

// Define Gains
float moment_of_inertia = 0.2; // [kg m^2]
float natural_frequency = 1; // [rad/s]
float damping_ratio = 1; // 0-1: Underdamped, 1: Critically Damped, 1+: Overdamped
float Kp = moment_of_inertia * natural_frequency * natural_frequency;
float Kd = 2 * damping_ratio * moment_of_inertia * natural_frequency; 
float gainIncrement = 0.01; //IR Remote stuff

// Thrust Characteristics based on 100 Psi Measurements
float thrust_output = 3.125; //thrust produced by a single nozzle [newtons]
float nozzle_moment_distance = .312; //perpindicular distance of a nozzle from the center-of-rotation of the platform [meters]
float control_torque = 2 * thrust_output * nozzle_moment_distance; // torque applied to the platform whenever a pair of thrusters is on [Nm]

// Declare Thrust Control Variables
float u; //control input [Nm]
float uCommanded; //bounded, signed control input [Nm]
float desired_thrust; //bounded, unsigned thrust from individual thruster [newtons]
bool pwpf_on; // true = thrusters allowed to fire (Schmidt ON), false = thrusters off
bool OnOffSwitch = false; // Define the boolean for On/Off state controlled by the IR Remote
unsigned long now; // [ms]
unsigned long lastPWPFUpdate; // [ms]
float dt;
float uAbs;
float pwpf_x;
float filtered_thrust; //bounded, unsigned thrust from individual thruster [newtons]
float Tf = 0.1; // [s]


// Define Schmitt Trigger Parameters
float deltaOn = 0.50; // [N]
float deltaOff= 0.25; // [N] deltaOff MUST BE SMALLER than deltaOn

// Setup for IMU
BNO080 myIMU;


//Setup for IR Remote
const uint32_t powerHex = 0xBA45FF00;
const uint32_t volUpHex = 0xB946FF00;
const uint32_t funcHex = 0xB847FF00;
const uint32_t rewindHex = 0xBB44FF00;
const uint32_t playPauseHex = 0xBF40FF00;
const uint32_t fastForwardHex = 0xBC43FF00;
const uint32_t downHex = 0xF807FF00;
const uint32_t volDownHex = 0xEA15FF00;
const uint32_t upHex = 0xF609FF00;
const uint32_t zeroHex = 0xE916FF00;
const uint32_t EQHex = 0xE619FF00;
const uint32_t STHex = 0xF20DFF00;
const uint32_t oneHex = 0xF30CFF00;
const uint32_t twoHex = 0xE718FF00;
const uint32_t threeHex = 0xA15EFF00;
const uint32_t fourHex = 0xF708FF00;
const uint32_t fiveHex = 0xE31CFF00;
const uint32_t sixHex = 0xA55AFF00;
const uint32_t sevenHex = 0xBD42FF00;
const uint32_t eightHex = 0xAD52FF00;
const uint32_t nineHex = 0xB54AFF00;




void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  delay(2000);
  Serial.println("Serial Begun");
  
  //Initialize the BNO085 IMU at address 0x4A, if not send error message
  if (!myIMU.begin(0x4A)) {
      while (1) {
          Serial.println(F("BNO085 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing..."));
          delay(5000);
      }
  }
  else Serial.println("MPU Connection Successful!"); Serial.println("MPU Connection Successful!"); Serial.println("MPU Connection Successful!"); Serial.println("MPU Connection Successful!"); delay(1000);
  myIMU.enableRotationVector(50); //Send data update every 50ms
  myIMU.enableGyro(50); //Send data update every 50ms

  //Initialize the IR Remote
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  // Set pin modes
  pinMode(solenoid1_pin, OUTPUT);
  pinMode(solenoid2_pin, OUTPUT);
  pinMode(remote_pin, INPUT);

  Serial.println();
  Serial.println("Starting..."); 
  Serial.println();
  Serial.println();

  updateIMUState();
  pwpf_on = false;
  pwpf_x = 0.0f;
  lastPWPFUpdate = millis();
}






void loop() {

  // Uncomment this for troubleshooting with the serial monitor
  /*
  if(OnOffSwitch == true)
  {
    Serial.print("Theta: ");Serial.print(theta);Serial.print(" \tTheta Dot: ");
    Serial.print(theta_dot);Serial.print(" \tDesired Thrust: ");
    Serial.print(desired_thrust); Serial.print(" \tCommanded Moment: ");
    Serial.print(u); Serial.print(" \tTime: ");
    Serial.println(millis()/1000);
  }
  else
  {
    Serial.println("Waiting to be activated...");
    Serial.print("Theta: ");Serial.print(theta);Serial.print(" \tTheta Dot: ");
    Serial.print(theta_dot);Serial.print(" \tDesired Thrust: ");
    Serial.print(desired_thrust); Serial.print(" \tCommanded Moment: ");
    Serial.print(u); Serial.print(" \tTime: ");
    Serial.println(millis()/1000);
    updateRemote();
  }
  */

  updateIMUState();
  updateRemote();

  desired_thrust = calculateDesiredThrust(theta, target_theta, theta_dot, target_theta_dot, Kp, Kd, control_torque); //desired, signed torque for an individual thruster

  updateIMUState();

  filtered_thrust = firstOrderFilter(desired_thrust); //applies first-order filter to smooth the control input before going into schmidt relay

  updateIMUState();

  fireThrustersSchmidt(filtered_thrust, desired_thrust);
}





// functions:

float calculateDesiredThrust(float theta, float target_theta, float theta_dot, float target_theta_dot, float Kp, float Kd, float control_torque)
{
  // Calculate Control Input through PD control law
  theta_error = PI*(target_theta - theta)/180; // [rad]
  theta_dot_error = PI*(target_theta_dot - theta_dot)/180; // [rad/s]
  u = Kp*theta_error+Kd*theta_dot_error; // [Nm]

  // if the desired control torque is greater than the actual control torque, constrain the signed commanded torque to be equal to control_torque
  if(abs(u)>control_torque){ 
    if(u>0) uCommanded = control_torque;
    else uCommanded = -control_torque;
  } 
  else uCommanded = u; // [Nm] signed, bounded torque

  updateIMUState();

  desired_thrust = uCommanded/nozzle_moment_distance/2; // [N] signed, bounded: represents the ideal output of a single thruster if we could throttle it (but we can't)

  return desired_thrust; // [newtons]
}

float firstOrderFilter(float uCmd)
{
  now = millis();
  dt = (now - lastPWPFUpdate) / 1000.0f; // [s]

  if (dt > 0.0f && dt < 0.5f) { // sanity bound dt
    float uAbs = fabs(uCmd); // [N], magnitude only
    pwpf_x += (dt / Tf) * (uAbs - pwpf_x);  // x_k+1 = x_k + dt/Tf (u - x)
  }
  lastPWPFUpdate = now;

  return pwpf_x; // [N]
}

void fireThrustersSchmidt(float filtered_mag, float uCmd)
{
  // 1) Schmitt on magnitude
  if (!pwpf_on && filtered_mag > deltaOn) {
    pwpf_on = true;
  } else if (pwpf_on && filtered_mag < deltaOff) {
    pwpf_on = false;
  }

  // 2) Apply gate + sign to thrusters
  if (OnOffSwitch && pwpf_on && fabs(uCmd) > 1e-6) {
    if (uCmd > 0) {
      // + torque
      digitalWrite(solenoid1_pin, LOW);
      digitalWrite(solenoid2_pin, HIGH);
    } else {
      // - torque
      digitalWrite(solenoid2_pin, LOW);
      digitalWrite(solenoid1_pin, HIGH);
    }
  } else {
    // OFF
    digitalWrite(solenoid1_pin, LOW);
    digitalWrite(solenoid2_pin, LOW);
  }
}

void updateRemote()
{
  if (IrReceiver.decode()) {
    uint32_t receivedValue = IrReceiver.decodedIRData.decodedRawData;

    // uncomment this for troubleshooting
    /*
    Serial.print("Received IR Code: 0x");
    Serial.println(receivedValue, HEX);
    */

    if (receivedValue == powerHex) {OnOffSwitch = !OnOffSwitch; digitalWrite(solenoid1_pin,LOW); digitalWrite(solenoid2_pin, LOW);}
    if (receivedValue == volUpHex) Kd += gainIncrement;
    if (receivedValue == volDownHex) Kd -= gainIncrement;
    if (receivedValue == upHex) Kp += gainIncrement;
    if (receivedValue == downHex) Kp -= gainIncrement;
    if (receivedValue == EQHex) target_theta -= 1;
    if (receivedValue == STHex) target_theta += 1;
    if (receivedValue == rewindHex) target_theta_dot -= 1;
    if (receivedValue == fastForwardHex) target_theta_dot += 1;

    IrReceiver.resume();  // Ready to receive the next value
  }
}

void updateIMUState()
{
  if (myIMU.dataAvailable() == true)
  {
    theta = myIMU.getYaw()/PI*180;
    theta_dot = myIMU.getGyroZ()/PI*180;
  }
  //Implement refresh of the OLED Screen
}