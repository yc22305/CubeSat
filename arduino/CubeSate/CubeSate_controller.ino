#include <Utility.h>

// cubesate and thruster properties
// assuming that center of mass is at geometric center
// float m = 16; // mass of cubesat
// float w = 0.239; // width of cubesat (meter)
float l = 0.464; // length of cubesat (meter)
// float I = 1/12*m*(w*w+l*l); // moment of inertia (kg*meter^2)
float thrust_M_design = 0.4*(l/2)*(3/4);// desirable moment in outerspace (without friction) produced by thruster
float thrust_M = (0.4 + 0.1)*(l/2)*(3/4); // moment in testing environment (on Earth) produced by thruster. Force is added to compensate the friction.
float on_duration_min = 0.03; // thruster restrcition of turning on
float off_duration_min = 0.03; // thrusster restrcition of turnning off
float Control_value_min = thrust_M_design*on_duration_min/(on_duration_min+off_duration_min);

float K_angle = 1.5; // feedback gain for angle position   
float K_angu_v = 1.2; // feedback gain for angular velocity

// controller related parameters
float uplimit = thrust_M_design; 
float deadband = 3/180*PI*K_angle; // within +-3 degree angles, thrusters are not activated.

float Expectation, Error, angle_sensor, angu_v_sensor;
float Control_value, duration_on, duration_cycle, time_last_on, time_last_off, currentTime;
float ProgramBeginTime; // used to record the beginning time of this program (second).
int thrust_switch; // status of thruster (on/off with direction)

void setup() {
  initModulator();
  currentTime = 0;
  ProgramBeginTime = millis()/1000;
}

void loop() {
  Expectation = 0; // the angle we want to track. Expectation = 0 represents pointing toward North.

  angle_sensor = getAngle();
  angu_v_sensor = getAnguV();
  Error = Expectation - angle_sensor*K_angle - angu_v_sensor*K_angu_v;

  // control value determination
  if (abs(Error) >= uplimit)  
     Control_value = sign(Error)*thrust_M_design;
  else
    {
     if (abs(Error) >= deadband) 
        Control_value = Error;
     else
        Control_value = 0;
    }

  // thruster status determination
  if (time_last_on > time_last_off) // thrusters are on.
    {
     if (abs(Control_value) >= Control_value_min)
        duration_on = off_duration_min*abs(Control_value)/(thrust_M_design-abs(Control_value));
     else
        duration_on = on_duration_min;

     currentTime = millis()/1000-ProgramBeginTime;
     if (currentTime-time_last_on < duration_on)
        thrust_switch = sign(Control_value);
     else
       {
        thrust_switch = 0;
        time_last_off = currentTime;
       }
    }        
  else // thrusters are off.
    {
     duration_cycle = thrust_M_design*duration_on/abs(Control_value);
     if (duration_cycle < duration_on+off_duration_min) // Minimum cycle due to thruster restrcition
        duration_cycle = duration_on+off_duration_min;

     currentTime = millis()/1000-ProgramBeginTime;
     if (currentTime < duration_cycle-duration_on)
         thrust_switch = 0;           
     else
        {
         thrust_switch = sign(Control_value);
         time_last_on = currentTime;
        }
    }

  activateThruster();
}

float getAngle()
{
  
}

float getAnguV()
{
  
}

void initModulator()
{
  Expectation = 0; // the angle we want to track. Expectation = 0 represents pointing toward North.

  angle_sensor = getAngle();
  angu_v_sensor = getAnguV();
  Error = Expectation - angle_sensor*K_angle - angu_v_sensor*K_angu_v;

  // control value determination
  if (abs(Error) >= uplimit)  
        Control_value = sign(Error)*thrust_M_design;
  else
     {
      if (abs(Error) >= deadband) 
          Control_value = Error;
      else
          Control_value = 0;
     }

  duration_on = on_duration_min;
  duration_cycle = thrust_M_design*duration_on/abs(Control_value);
  if (duration_cycle < duration_on+off_duration_min)
     duration_cycle = duration_on+off_duration_min;
  time_last_on = -duration_cycle;
  time_last_off = -duration_cycle+duration_on;
}

void activateThruster()
{
  
}

