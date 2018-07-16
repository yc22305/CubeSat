/*rosserial_arduino package should be installed*/

#define _SAM3XA_ // For arduino DUE
#define USE_SERIAL_ONE // using Serial1 to transmit ROS messages.
#define USE_USBCON // For arduino except Leonardo version

#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C_Wire1.h>
#include <Utility.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/SetBool.h>
#include <serial_srvs/DesiredValue.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

#define BAUD 115200 // baudrate of transmitting ROS messages.
#define CONTROL_PERIOD 200 // set the time interval (millisecond) between each control determination
#define ROS_REPORT_PERIOD 50 // set the time interval (millisecond) between each ROS message publishing
#define RANGE_ANGLE_ADJUST 175 // around -180 and 180 (discontinuity) degrees, angle representation should be adjusted to 0~360 to fit the algorithm.  

// Relays
#define RELAY_POWER 13 // relay one
#define RELAY_NEGA_YAW 10 // relay two
#define RELAY_MAIN 11 // relay three
#define RELAY_POSI_YAW 12 // relay four

#define SerialDebug false   // set to true to get Serial output for debugging
#define CALIBRATION false // set true to do MPU9250 calibration. when calibrating, you can open Arduino Serial Monitor to check the stage and get bias data.
#define LCD true // set true to print on LCD

// Using I2C monochrome LCD
LiquidCrystal_I2C display(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// CubeSat modes
/* -1 : Wait for adjusting wiring to connect to IMU
   0 : Power off the thruster. Only read sensor data. No control determination.
   1 : Power on the CubeSat*/
int CubeMode; 

uint32_t delt_t = 0, delt_t_ros = 0; // rate of control value determination
uint32_t count = 0, count_ros = 0; // used to control display output rate
float deltat = 0.0f, sum = 0.0f; // time interval for the filter scheme
uint32_t lastUpdate = 0, Now = 0, sumCount = 0; // used to calculate rate of the filter scheme

#define MAG_DECLINE_RAD -3.5f/180*PI // magmetic decline in Tainan

float pitch, yaw, roll;
float ax, ay, az, gx, gy, gz, mx, my, mz, mx_temp, my_temp, mz_temp; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
int16_t tempCount;      // temperature raw count output
float temperature;    // Stores the real internal chip temperature in degrees Celsius

// cubesate and thruster properties
// assuming that center of mass is at geometric center
// #define w 0.239f; // width of cubesat (meter)
#define l 0.464f // length of cubesat (meter)
#define thrust_M_design 0.4f*(l/2.0f)*(3/4.0f)// desirable moment in outerspace (without friction) produced by thruster
#define on_duration_min 0.03f // thruster restrcition of turning on
#define off_duration_min 0.03f // thrusster restrcition of turnning off
#define Control_value_min thrust_M_design*on_duration_min/(on_duration_min+off_duration_min)

//// controller related parameters
#define K_angle 0.3f // feedback gain for angle position   
#define K_angu_v 0.6f // feedback gain for angular velocity
#define uplimit thrust_M_design 
#define deadband uplimit*0.25f // this deadband is determined by experiments.
#define Expectation 0 // The desired value of the control loop, consisting of the current angle and angular velocity. Expectation = 0 should not be changed.

float DesiredValue = 0, DesiredValue_input = 0; // the angle (degree) we want to track. Angle toward north is 0. DesiredValue_input is to store the user command, while DesiredValue is adjusted and for the program to calculate with.

float Error, angle_sensor, angu_v_sensor, Control_value, duration_on, duration_cycle, time_last_on, time_last_off, currentTime = 0;
float ProgramBeginTime; // used to record the beginning time of this program (second).
int thrust_switch = 0; // status (thrust direction) of thruster

//// ros callback functions
void getDesiredValue(const serial_srvs::DesiredValue::Request &req, serial_srvs::DesiredValue::Response &res)
{
  DesiredValue_input = req.data;
  if (DesiredValue_input < -RANGE_ANGLE_ADJUST)
     DesiredValue = DesiredValue_input+360; // adjust -180~0 to 180~360 when DesiredValue_input is around +-180
  else
     DesiredValue = DesiredValue_input;
  //res.message = "succesfully send desired value!";
}

void powerThruster(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if (req.data) { // power the thruster
     CubeMode = 1;
     digitalWrite(RELAY_POWER, HIGH);
     //res.message = "successfully power on thrusters!";
    }
  else {
     CubeMode = 0;
     digitalWrite(RELAY_POWER, LOW);
     //res.message = "successfully power off thrusters!";
    }
}

ros::NodeHandle nh;

geometry_msgs::TransformStamped transf; // to store the attitude and position of the cubesat
std_msgs::Float32 debug_desiredValue_msg;
std_msgs::Int16 debug_thrustSwitch_msg;
std_msgs::Int16 debug_cubeMode_msg;

tf::TransformBroadcaster broadcaster;
ros::ServiceServer<serial_srvs::DesiredValue::Request, serial_srvs::DesiredValue::Response> desiredValue_server("desired_value", &getDesiredValue);
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> thrustPowered_server("thrust_powered", &powerThruster);
ros::Publisher debug_desiredValue_pub("debug_desiredValue", &debug_desiredValue_msg);
ros::Publisher debug_thrustSwitch_pub("debug_thrustSwitch", &debug_thrustSwitch_msg);
ros::Publisher debug_cubeMode_pub("debug_cubeMode", &debug_cubeMode_msg);

void setup()
{
  nh.getHardware() -> setBaud(BAUD);
  nh.initNode();
  nh.advertiseService(desiredValue_server);
  nh.advertiseService(thrustPowered_server);
  nh.advertise(debug_desiredValue_pub);
  nh.advertise(debug_thrustSwitch_pub);
  nh.advertise(debug_cubeMode_pub);

  broadcaster.init(nh);
  transf.header.frame_id = "/world";
  transf.child_frame_id = "/cubesat";

  pinMode(RELAY_POWER, OUTPUT);
  pinMode(RELAY_NEGA_YAW, OUTPUT);
  pinMode(RELAY_MAIN, OUTPUT);
  pinMode(RELAY_POSI_YAW, OUTPUT);
  digitalWrite(RELAY_POWER, LOW);
  digitalWrite(RELAY_NEGA_YAW, LOW);
  digitalWrite(RELAY_MAIN, LOW);
  digitalWrite(RELAY_POSI_YAW, LOW);
  
  Wire.begin();
  if (SerialDebug | CALIBRATION) {
     Serial.begin(115200);
    }

  if (LCD) {
    display.init(); // Initialize the LCD
    display.backlight();
  }
 
  setupIMU(); // will determine the initial CubeSat mode
  initModulator(); 
  ProgramBeginTime = millis()/1000;
}

void loop()
{ 
  if (CubeMode == 0 | CubeMode == 1) { // Successfully connects
     readNewData(); // read acceleration, gyro rate, magnetic field and temperature.
  
     Now = micros();
     deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update. using micros() to get more acurate data.
     lastUpdate = Now;

     sum += deltat; // sum for averaging filter update rate
     sumCount++;
  
     // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
     // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
     // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
     // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
     // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
     // This is ok by aircraft orientation standards!  
     // Pass gyro rate as rad/s
     //MadgwickQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f, my, -mx, mz);
     MahonyQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f, my, -mx, mz);
  
     delt_t = millis() - count;
     if (delt_t > CONTROL_PERIOD) { // set control value per 0.5 seconds independent of read rate
        if (SerialDebug) {    
           Serial.print("ax = "); Serial.print((int)1000*ax);  
           Serial.print(" ay = "); Serial.print((int)1000*ay); 
           Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
           Serial.print("gx = "); Serial.print( gx, 2); 
           Serial.print(" gy = "); Serial.print( gy, 2); 
           Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
           Serial.print("mx = "); Serial.print( (int)mx ); 
           Serial.print(" my = "); Serial.print( (int)my ); 
           Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
           Serial.print("Temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C");
    
           Serial.print("q0 = "); Serial.print(q[0]);
           Serial.print(" qx = "); Serial.print(q[1]); 
           Serial.print(" qy = "); Serial.print(q[2]); 
           Serial.print(" qz = "); Serial.println(q[3]); 
          } 
              
        // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
        // In this coordinate system, the positive z-axis is down toward Earth. 
        // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
        // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is negative, up toward the sky is positive.
        // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
        // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
        // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
        // applied in the correct order which for this configuration is yaw, pitch, and then roll.
        // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
        yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
        pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        yaw   *= 180.0f / PI; 
        pitch *= 180.0f / PI;
        roll  *= 180.0f / PI;              

        // control value determination
        if (CubeMode == 1) {
           angle_sensor = getAngle() - DesiredValue/180*PI;
           angu_v_sensor = getAnguV();
           Error = Expectation - angle_sensor*K_angle - angu_v_sensor*K_angu_v;
        
           if (abs(Error) >= uplimit)  
              Control_value = sign(Error)*thrust_M_design;
           else {
              if (abs(Error) >= deadband) 
                 Control_value = Error;
              else
                 Control_value = 0;
             }
        
           // thruster status determination
           if (time_last_on > time_last_off) { // thrusters are on.
              if (abs(Control_value) >= Control_value_min)
                 duration_on = off_duration_min*abs(Control_value)/(thrust_M_design-abs(Control_value));
              else
                 duration_on = on_duration_min;

              currentTime = millis()/1000-ProgramBeginTime;
              if (currentTime-time_last_on < duration_on)
                 thrust_switch = sign(Control_value);
              else {
                 thrust_switch = 0;
                 time_last_off = currentTime;
                }
             }        
           else { // thrusters are off.
              duration_cycle = thrust_M_design*duration_on/abs(Control_value);
              if (duration_cycle < duration_on+off_duration_min) // Minimum cycle due to thruster restrcition
                 duration_cycle = duration_on+off_duration_min;

                 currentTime = millis()/1000-ProgramBeginTime;
              if (currentTime < duration_cycle-duration_on)
                 thrust_switch = 0;           
              else {
                 thrust_switch = sign(Control_value);
                 time_last_on = currentTime;
                }
             }           
          }
        if (CubeMode == 0) {
           thrust_switch = 0;
          }  
        switchThruster();
           
        if (SerialDebug) {
           Serial.print("Yaw, Pitch, Roll: ");
           Serial.print(yaw, 1);
           Serial.print(", ");
           Serial.print(pitch, 1);
           Serial.print(", ");
           Serial.println(roll, 1);
    
           Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
          }

        if (LCD) {
           display.clear();
           display.print("Yaw  Pitch  Roll");
           display.setCursor(0,1); display.print(yaw, 0);
           display.setCursor(5,1); display.print(pitch, 0);
           display.setCursor(12,1); display.print(roll, 0);
          }

        count = millis(); 
         
        sumCount = 0;
        sum = 0;    
       }
    }

  // ROS message publishers. 
  delt_t_ros = millis() - count_ros;
  if (delt_t_ros > ROS_REPORT_PERIOD) {
     debug_cubeMode_msg.data = CubeMode;
     debug_desiredValue_msg.data = DesiredValue_input;
     debug_thrustSwitch_msg.data = thrust_switch;

     debug_cubeMode_pub.publish(&debug_cubeMode_msg);
     debug_desiredValue_pub.publish(&debug_desiredValue_msg);
     debug_thrustSwitch_pub.publish(&debug_thrustSwitch_msg);

     transf.transform.translation.x = 0.0;
     transf.transform.translation.y = 0.0;
     transf.transform.translation.z = 0.0; 
     transf.transform.rotation.x = q[1];
     transf.transform.rotation.y = q[2]; 
     transf.transform.rotation.z = q[3]; 
     transf.transform.rotation.w = q[0];  
     transf.header.stamp = nh.now();
     broadcaster.sendTransform(transf);
         
     count_ros = millis(); 
    }
 
  nh.spinOnce();
}

float getAngle()
{
  if (DesiredValue_input < -RANGE_ANGLE_ADJUST | DesiredValue_input > RANGE_ANGLE_ADJUST) {
     if (yaw < 0)
        return (yaw+360)/180*PI; // adjust -180~0 to 180~360
     else
        return yaw/180*PI;
    }
  else
     return yaw/180*PI;
}

float getAnguV()
{
  return -gz/180*PI; // In the coordinate applied by gyro, z-axis is outward Earth. To comform the referred frame, z-axis is inverse.
}

void initModulator()
{
  duration_on = on_duration_min;
  duration_cycle = duration_on+off_duration_min;
  time_last_on = -duration_cycle;
  time_last_off = -duration_cycle+duration_on;
}

void switchThruster()
{
 if (thrust_switch==0) {
    digitalWrite(RELAY_NEGA_YAW, LOW);
    digitalWrite(RELAY_POSI_YAW, LOW);
   }
 if (thrust_switch==1) {
    digitalWrite(RELAY_NEGA_YAW, LOW);
    digitalWrite(RELAY_POSI_YAW, HIGH);
   }
 if (thrust_switch==-1) {
    digitalWrite(RELAY_NEGA_YAW, HIGH);
    digitalWrite(RELAY_POSI_YAW, LOW);
   }
}

