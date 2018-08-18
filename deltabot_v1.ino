
#include <Servo.h>
#include <math.h>

// Robot physical parameters in meters (ABB delta robot)
#define L     0.524
#define l     1.244
#define w_b   0.164
#define u_b   0.327
#define s_b   0.567
#define w_p   0.022
#define u_p   0.044
#define s_p   0.076

// Pin definitions 
#define PWM_PIN1  9
#define PWM_PIN2  10
#define PWM_PIN3  11
#define GRIP_PIN  6

// Constants definitions
#define GRIP_CLOSE  7
#define GRIP_OPEN   90

Servo servo1;
Servo servo2;
Servo servo3;
Servo gripper;

struct Angles {
  double theta1;
  double theta2;
  double theta3;
};

void close_gripper()
{
  gripper.write(GRIP_CLOSE);
}

void open_gripper()
{
  gripper.write(GRIP_OPEN);
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
  
  return ret;
}

void setup() 
{
  // Attach servos to their respective pins
  servo1.attach(PWM_PIN1);
  servo2.attach(PWM_PIN2);
  servo3.attach(PWM_PIN3);
  gripper.attach(GRIP_PIN);

  // Move servos to initial positions
  servo1.write(0);
  servo2.write(0);
  servo3.write(0);
  close_gripper();

  Serial.begin(9600);
}

void loop() 
{
  servo1.write(0);
  servo2.write(0);
  servo3.write(0);
  delay(1500);
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  delay(1500);
}





