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
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

#define BAUD 115200 // baudrate of transmitting messages.
#define CONTROL_PERIOD 200 // set the time interval (millisecond) between each control determination
#define ROS_REPORT_PERIOD 50 // set the time interval (millisecond) between each ROS message publishing
#define RANGE_ANGLE_ADJUST 175 // around -180 and 180 (discontinuity) degrees, angle representation should be adjusted to 0~360 to fit the algorithm.

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
// Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Using the MSENSR-9250 breakout board, ADO is set to 0 
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 1
#if ADO
#define MPU9250_ADDRESS 0x69  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#endif  

#define AHRS true         // set to false for basic data read
#define SerialDebug true   // set to true to get Serial output for debugging
#define MAG_CALIBRATION true // set true to do magnetic calibration
#define LCD true // set true to print on LCD

// Using I2C LCD monochrome 84 x 48 pixel display
LiquidCrystal_I2C display(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3] = {0, 0, 0};  // Factory mag calibration, mag bias, and bias scale factor
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;      // temperature raw count output
float temperature;    // Stores the real internal chip temperature in degrees Celsius
float SelfTest[6];    // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
//float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
//float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
//float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
//float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, delt_t_ros = 0; // rate of control value determination
uint32_t count = 0, count_ros = 0; // used to control display output rate
float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f; // time interval for the filter scheme
uint32_t lastUpdate = 0, Now = 0, sumCount = 0; // used to calculate rate of the filter scheme

float ax, ay, az, gx, gy, gz, mx, my, mz, mx_temp, my_temp, mz_temp; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

#define MAG_DECLINE_RAD -3.5f/180*PI // magmetic decline in Tainan

// cubesate and thruster properties
// assuming that center of mass is at geometric center
// float w = 0.239f; // width of cubesat (meter)
float l = 0.464f; // length of cubesat (meter)
float thrust_M_design = 0.4f*(l/2.0f)*(3/4.0f);// desirable moment in outerspace (without friction) produced by thruster
float on_duration_min = 0.03f; // thruster restrcition of turning on
float off_duration_min = 0.03f; // thrusster restrcition of turnning off
float Control_value_min = thrust_M_design*on_duration_min/(on_duration_min+off_duration_min);
bool ifPowerThruster;

//// controller related parameters
float K_angle = 0.3f; // feedback gain for angle position   
float K_angu_v = 0.6f; // feedback gain for angular velocity
float uplimit = thrust_M_design; 
float deadband = uplimit*0.2f; // this deadband is determined by experiments.

float Expectation = 0; // The desired value of the control loop, compared with the sum of the current angle and angular velocity. Expectation = 0 should not be changed. 
float DesiredValue, DesiredValue_input; // the angle (degree) we want to track. Angle toward north is 0. DesiredValue_input is to store the user command, while DesiredValue is adjusted and for the program to calculate with.

float Error, angle_sensor, angu_v_sensor, Control_value, duration_on, duration_cycle, time_last_on, time_last_off, currentTime;
float ProgramBeginTime; // used to record the beginning time of this program (second).
int thrust_switch = 0; // status (thrust direction) of thruster

//// ros callback functions
void getDesiredValue(const serial_srvs::DesiredValue::Request &req, serial_srvs::DesiredValue::Response &res)
{
  DesiredValue_input = req.data;
  if (DesiredValue_input < -RANGE_ANGLE_ADJUST | DesiredValue_input > RANGE_ANGLE_ADJUST)
     DesiredValue = DesiredValue_input+360; // adjust -180~0 to 180~360
  else
     DesiredValue = DesiredValue_input;
  res.message = "succesfully send desired value!";
}

void powerThruster(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if (req.data) { // power the thruster
     ifPowerThruster = true;
     res.message = "successfully power on thrusters!";
    }
  else {
     ifPowerThruster = false;
     res.message = "successfully power off thrusters!";
    }
}

ros::NodeHandle nh;
geometry_msgs::TransformStamped transf; // to store the attitude and position of the cubesat
std_msgs::String debug_thrustPowered_msg;
std_msgs::Float32 debug_desiredValue_msg;
std_msgs::Int16 debug_thrustSwitch_msg;
//std_msgs::String debug_anyMsg_msg;

tf::TransformBroadcaster broadcaster;
ros::ServiceServer<serial_srvs::DesiredValue::Request, serial_srvs::DesiredValue::Response> desiredValue_server("desired_value", &getDesiredValue);
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> thrustPowered_server("thrust_powered", &powerThruster);
ros::Publisher debug_thrustPowered_pub("debug_thrustPowered", &debug_thrustPowered_msg);
ros::Publisher debug_desiredValue_pub("debug_desiredValue", &debug_desiredValue_msg);
ros::Publisher debug_thrustSwitch_pub("debug_thrustSwitch", &debug_thrustSwitch_msg);
//ros::Publisher debug_anyMsg_pub("debug_anyMsg", &debug_anyMsg_msg);

void setup()
{
  nh.getHardware() -> setBaud(BAUD);
  nh.initNode();
  nh.advertiseService(desiredValue_server);
  nh.advertiseService(thrustPowered_server);
  nh.advertise(debug_thrustPowered_pub);
  nh.advertise(debug_desiredValue_pub);
  nh.advertise(debug_thrustSwitch_pub);
  //nh.advertise(debug_anyMsg_pub);
  broadcaster.init(nh);
  
  transf.header.frame_id = "/world";
  transf.child_frame_id = "/cubesat";
  
  Wire.begin();
  if (SerialDebug) {
     Serial.begin(BAUD);
    }

  if (LCD) {
    display.init(); // Initialize the LCD
    display.backlight();
  }

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  if (SerialDebug) {
      Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
     }
  delay(1000); 

  if (c == 0x71) { // WHO_AM_I should always be 0x71
     if (SerialDebug) { 
         Serial.println("MPU9250 is online...");
        }
     if (LCD) {
        display.clear();
        display.setCursor(0,0);
        display.print("MPU9250");
        display.setCursor(0,1); display.print("Calibrating..");
       }
      
     getMres();
     getGres();
     getAres();

     if (SerialDebug) {
        MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
        Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
        Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
        Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
        Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
        Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
        Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value"); 
       }

     calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
     if (SerialDebug) {
        Serial.print("gyroBias-x: "); Serial.println(gyroBias[0]);
        Serial.print("gyroBias-y: "); Serial.println(gyroBias[1]);
        Serial.print("gyroBias-z: "); Serial.println(gyroBias[2]);
        Serial.print("accelBias-x: "); Serial.println(accelBias[0]);
        Serial.print("accelBias-y: "); Serial.println(accelBias[1]);
        Serial.print("accelBias-z: "); Serial.println(accelBias[2]);
       }
     initMPU9250(); 
     delay(1000); 

     // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
     byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
     if (SerialDebug) {
        Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
       }
     delay(1000); 

     if (d == 0x48) { // WHO_AM_I should always be 0x48
        if (SerialDebug) { 
           Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
          }
        initAK8963(magCalibration); 
        if (LCD) {
           display.clear();
           display.print("AK8963");
           display.setCursor(0,1); display.print("Calibrating..");
          }

        if (MAG_CALIBRATION)
            magcalMPU9250(magBias, magScale);
        else {
            magBias[0] = -86.03; // determined by previous test of the magnetometer
            magBias[1] = 228.66;
            magBias[2] = -371.48;
            magScale[0] = 1.08;
            magScale[1] = 1.08;
            magScale[2] = 0.88;   
           }
        delay(1000);

        ifPowerThruster = false;
        DesiredValue = 0; // DesiredValue = 0 represents pointing toward North.
        initModulator(); // must be executed after the desired value is set.
        currentTime = 0;
        ProgramBeginTime = millis()/1000;
       }
     else {
         if (SerialDebug) {
             Serial.print("Could not connect to AK8963: 0x");
             Serial.println(d, HEX);
           }
         if (LCD) {
             display.clear();
             display.print("CAN'T CONNECT");
             display.setCursor(0,1); display.print("TO AK8963!");
           }
         while (1); // fails to connect; waiting for adjusting
        }
    }
  else {
      if (SerialDebug) {
         Serial.print("Could not connect to MPU9250: 0x");
         Serial.println(c, HEX);
        }
      if (LCD) {
         display.clear();
         display.print("CAN'T CONNECT");
         display.setCursor(0,1); display.print("TO MPU9250!");
        }
      while (1); // fails to connect; waiting for adjusting
     }
}

void loop()
{   
  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
     readAccelData(accelCount);  // Read the x/y/z adc values
     ax = (float)accelCount[0]*aRes - accelBias[0];  // get the actual g value, this depends on scale being set
     ay = (float)accelCount[1]*aRes - accelBias[1];   
     az = (float)accelCount[2]*aRes - accelBias[2];  
   
     readGyroData(gyroCount);  // Read the x/y/z adc values 
     // Calculate the gyro value into actual degrees per second
     gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
     gy = (float)gyroCount[1]*gRes;  
     gz = (float)gyroCount[2]*gRes;   

     readMagData(magCount);  // Read the x/y/z adc values    
     mx_temp = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get the actual magnetometer value, this depends on scale being set
     my_temp = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
     mz_temp = (float)magCount[2]*mRes*magCalibration[2] - magBias[2]; 
     mx_temp *= magScale[0];
     my_temp *= magScale[1];
     mz_temp *= magScale[2];
     // calibrate the magnetic decline
     mx = cos(-MAG_DECLINE_RAD)*mx_temp - sin(-MAG_DECLINE_RAD)*my_temp;
     my = sin(-MAG_DECLINE_RAD)*mx_temp + cos(-MAG_DECLINE_RAD)*my_temp;
     mz = mz_temp;
    }
  
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
  
  if (!AHRS) {
      delt_t = millis() - count;
      if (delt_t > 500) {
         if (SerialDebug) {
            // Print acceleration values in milligs!
            Serial.print("X-acceleration: "); Serial.print(1000*ax); Serial.print(" mg ");
            Serial.print("Y-acceleration: "); Serial.print(1000*ay); Serial.print(" mg ");
            Serial.print("Z-acceleration: "); Serial.print(1000*az); Serial.println(" mg ");
 
            // Print gyro values in degree/sec
            Serial.print("X-gyro rate: "); Serial.print(gx, 3); Serial.print(" degrees/sec "); 
            Serial.print("Y-gyro rate: "); Serial.print(gy, 3); Serial.print(" degrees/sec "); 
            Serial.print("Z-gyro rate: "); Serial.print(gz, 3); Serial.println(" degrees/sec"); 
    
            // Print mag values in degree/sec
            Serial.print("X-mag field: "); Serial.print(mx); Serial.print(" mG "); 
            Serial.print("Y-mag field: "); Serial.print(my); Serial.print(" mG "); 
            Serial.print("Z-mag field: "); Serial.print(mz); Serial.println(" mG"); 
 
            tempCount = readTempData();  // Read the adc values
            temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
            // Print temperature in degrees Centigrade      
            Serial.print("Temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
           }
    
        count = millis();
       }
    }
  else {
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

         // controller
         angle_sensor = getAngle() - DesiredValue/180*PI;
         angu_v_sensor = getAnguV();
         Error = Expectation - angle_sensor*K_angle - angu_v_sensor*K_angu_v;
       /*  if (SerialDebug) {
            Serial.print("deadband: "); Serial.println(deadband);
            Serial.print("thrust_M_design: "); Serial.println(thrust_M_design);
            Serial.print("Uplimit: "); Serial.println(uplimit);
            Serial.print("Error: "); Serial.println(Error);
           }*/

         // control value determination
         if (abs(Error) >= uplimit)  
            Control_value = sign(Error)*thrust_M_design;
         else {
            if (abs(Error) >= deadband) 
               Control_value = Error;
            else
               Control_value = 0;
           }

        /* if (SerialDebug) {
            Serial.print("Control value: "); Serial.println(Control_value);
           }*/

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
        
      delt_t_ros = millis() - count_ros;
      if (delt_t_ros > ROS_REPORT_PERIOD) {
         if (ifPowerThruster)
            debug_thrustPowered_msg.data = "ON";
         else
            debug_thrustPowered_msg.data = "OFF";
         debug_desiredValue_msg.data = DesiredValue_input;
         debug_thrustSwitch_msg.data = thrust_switch;

         debug_thrustPowered_pub.publish(&debug_thrustPowered_msg);
         debug_desiredValue_pub.publish(&debug_desiredValue_msg);
         debug_thrustSwitch_pub.publish(&debug_thrustSwitch_msg);
         //debug_anyMsg_pub.publish(&debug_anyMsg_msg);

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
  return -gz/180*PI; // In the coordinate applied by gyro, z-axis is outward Earth. To comform the refered frame, z-axis is inverse.
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
 /* Serial.println(thrust_switch);
  Serial.println("Thrust!"); */
}

