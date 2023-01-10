#include <SimpleFOC.h>
#include <PID_v1.h>

//My motor pins (13, 12, 14, 27)
//pp = 7

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(13, 12, 14, 27);

// voltage set point variable
float target_voltage = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }

void setup() {

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // power supply voltage
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // aligning voltage 
  motor.voltage_sensor_align = 5;
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor.controller = MotionControlType::angle;
   //motor.controller = MotionControlType::angle;

   // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20; //20
  motor.PID_velocity.D = 0;
  // maximal voltage to be set to the motor
  motor.voltage_limit = 6;

  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f; //0.01f

  // angle P controller
  motor.P_angle.P = 20; //20
  // maximal velocity of the position control
  motor.velocity_limit = 40;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target voltage");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target voltage using serial terminal:"));
  _delay(1000);
}

void loop() {

  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code

  if (sensor.getAngle() < 1.5) {
    //motor.controller = MotionControlType::angle;
    //motor.PID_velocity.P = 0.2f;
    //motor.PID_velocity.I = 20; //20;
    motor.move(2);
  } else if (sensor.getAngle() >= 1.50) {
    //motor.controller = MotionControlType::angle;
    //motor.PID_velocity.P = 0.2f;
   // motor.PID_velocity.I = 20; //20;
    motor.move(2);
  } /*else {
    motor.controller = MotionControlType::torque;
   // motor.PID_velocity.P = 0;
   // motor.PID_velocity.I = 0; //20
    motor.move(0);
  }
  */
  Serial.println(sensor.getAngle());
//  motor.velocity(0);
  
  // user communication
  //command.run();
}
