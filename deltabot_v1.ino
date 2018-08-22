
#include <Servo.h>
#include <math.h>

/* 
 *  Robot physical parameters in meters 
 *  (See 'The Delta Parallel Robot: Kinematics Solutions - Robert L. Williams II'
 *   for reference)
 */
// ABB Flexpicker delta robot
//#define L     0.524
//#define l     1.244
//#define w_b   0.164
//#define u_b   0.327
//#define s_b   0.567
//#define w_p   0.022
//#define u_p   0.044
//#define s_p   0.076

// EEZYbotDELTA
#define L     0.0956
#define l     0.2200
#define w_b   0.0666
#define u_b   0.1240
#define s_b   0.2092
#define w_p   0.0431
#define u_p   0.0821
#define s_p   0.1412

// Pin definitions 
#define PWM_PIN1  9
#define PWM_PIN2  10
#define PWM_PIN3  11
#define GRIP_PIN  6

// Constants definitions
#define GRIP_CLOSE    7
#define GRIP_OPEN     90
#define SERVO1_OFFSET 35
#define SERVO2_OFFSET 21
#define SERVO3_OFFSET 16

// Robot's servos definitions
Servo servo1;
Servo servo2;
Servo servo3;
Servo gripper;

struct Angles {
  double theta1;
  double theta2;
  double theta3;
};

// Robot position
double pos_x;
double pos_y;
double pos_z;

/*
 * Close the robot's end effector
 */
void closeGripper()
{
  gripper.write(GRIP_CLOSE);
}

/*
 * Open the robot's end effector
 */
void openGripper()
{
  gripper.write(GRIP_OPEN);
}

/*
 * Map an angle value to a microseconds value for later use with the Servo library's
 * 'writeMicroseconds'
 */
long angleToMicrosecs(float angle)
{
  return map((long) angle * 10, 0, 1800, 700, 2300);
}

/*
 * Command for bringing the gripper back at its home position
 */
void moveHome()
{
  moveToPoint(0, 0, -0.15);
}

/*
 * Set the robot's servos according to angles given as arguments
 */
void setServos(float angle1, float angle2, float angle3)
{
  // Check angles' validity
  if(angle1 < 0 || angle1 > 150 || isnan(angle1))
  {
    Serial.print("[ERROR] Invalid servo 2 angle : "); Serial.println(angle1);
    return;
  }

  if(angle2 < 0 || angle2 > 150 || isnan(angle2))
  {
    Serial.print("[ERROR] Invalid servo 2 angle : "); Serial.println(angle2);
    return;
  }

  if(angle3 < 0 || angle3 > 150 || isnan(angle3))
  {
    Serial.print("[ERROR] Invalid servo 3 angle : "); Serial.println(angle3);
    return;
  }

  Serial.print("[SET]\t");
  Serial.print(angle1); Serial.print("\t");
  Serial.print(angle2); Serial.print("\t");
  Serial.println(angle3);
  servo1.writeMicroseconds(angleToMicrosecs(angle1 + SERVO1_OFFSET));
  servo2.writeMicroseconds(angleToMicrosecs(angle2 + SERVO2_OFFSET));
  servo3.writeMicroseconds(angleToMicrosecs(angle3 + SERVO3_OFFSET));
}

/*
 * Compute servo angles using analytical inverse kinematics equations
 * 
 * Ref: 'The Delta Parallel Robot: Kinematics Solutions - Robert L. Williams II'
 */
Angles inverseKinematics(float x, float y, float z)
{
  // Initialize return variable
  Angles ret;
  
  // Compute constants
  double a = w_b - u_p;
  double b = (s_p/2) - ((sqrt(3)/2) * w_b);
  double c = w_p - (0.5 * w_b);

  // Compute angle specific values
  double F = 2 * z * L;
  double common_G = square(x) + square(y) + square(z) + square(L) - square(l);

  double E_1 = 2 * L * (y + a);
  double G_1 = common_G + square(a) + (2 * y * a);

  double E_2 = -L * ((sqrt(3) * (x + b)) + y + c);
  double G_2 = common_G + square(b) + square(c) + (2 * ((x * b) + (y * c)));

  double E_3 = L * (sqrt(3) * (x - b) - y - c);
  double G_3 = common_G + square(b) + square(c) + (2 * ((- x * b) + (y * c)));

  // Compute angles
  ret.theta1 = 2 * atan2(-F - sqrt(square(E_1) + square(F) - square(G_1)), G_1 - E_1);
  ret.theta2 = 2 * atan2(-F - sqrt(square(E_2) + square(F) - square(G_2)), G_2 - E_2);
  ret.theta3 = 2 * atan2(-F - sqrt(square(E_3) + square(F) - square(G_3)), G_3 - E_3);

  // Convert to degrees
  ret.theta1 *= (180.0f / PI);
  ret.theta2 *= (180.0f / PI);
  ret.theta3 *= (180.0f / PI);
  
  return ret;
}

/*
 * Convert angles produced by inverse kinematics into actual servo orientation angles 
 */
Angles cleanAngles(Angles dirty)
{
  Angles ret;

  ret.theta1 = 90.0f - dirty.theta1;
  ret.theta2 = 90.0f - dirty.theta2;
  ret.theta3 = 90.0f - dirty.theta3;

  if(ret.theta1 > 180)
    ret.theta1 -= 360;
  else if(ret.theta1 < 0)
    ret.theta1 += 360;

  if(ret.theta2 > 180)
    ret.theta2 -= 360;
  else if(ret.theta2 < 0)
    ret.theta2 += 360;

  if(ret.theta3 > 180)
    ret.theta3 -= 360;
  else if(ret.theta3 < 0)
    ret.theta3 += 360;

  return ret;
}

/*
 * Given a cartesian point expressed in the robot's frame of reference,
 * move the robot's platform to that position
 */
void moveToPoint(float x, float y, float z)
{
  Angles res = cleanAngles(inverseKinematics(x, y, z));
  setServos(res.theta1, res.theta2, res.theta3);
  pos_x = x; 
  pos_y = y;
  pos_z = z;
}

/*
 * Move the robot's gripper to the specified position along a line from the initial
 * position to the new point
 * 
 * Arguments:
 *  -num_points:  Number of points generated from the line
 */
void lineToPoint(float x, float y, float z, int num_points)
{
  double init_x = pos_x;
  double init_y = pos_y;
  double init_z = pos_z;
  
  double inc_x = (x - pos_x) / ((float) num_points);
  double inc_y = (y - pos_y) / ((float) num_points);
  double inc_z = (z - pos_z) / ((float) num_points);

  for(int i = 1; i <= num_points; ++i)
    moveToPoint(init_x + (i * inc_x), init_y + (i * inc_y), init_z + (i * inc_z));
}

void setup() 
{
  // Attach servos to their respective pins
  servo1.attach(PWM_PIN1);
  servo2.attach(PWM_PIN2);
  servo3.attach(PWM_PIN3);
  gripper.attach(GRIP_PIN);

  // Move servos to initial positions
  moveHome();
  closeGripper();
  delay(1000);

  Serial.begin(9600);
}

void loop()
{
  lineToPoint(0.05, 0, -0.2, 50);
  delay(100);
  lineToPoint(0.05, 0, -0.25, 50);
  delay(100);
  lineToPoint(-0.05, 0, -0.25, 50);
  delay(100);
  lineToPoint(-0.05, 0, -0.2, 50);
  delay(100);
}

