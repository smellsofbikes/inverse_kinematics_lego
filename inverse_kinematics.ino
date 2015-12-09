/*
This program controls a robot arm like that discussed at the link below.
http://projectsfromtech.blogspot.com/2013/09/simple-arduino-robot-arm-from-9-gram.html

Position is calculated using simple inverse kinematics based on high school trig

Setup:
* Change arm constants to match physical setup (distance between pivot points in cm)
* Change servo correction factors to match physical setup
* Change origin correction factors to match desired origin

Use:
* Enter (x,y) coordinates in FixCoordinate(x,y). Distances are in cm and are measured from origin
* Call CalculateServoAngles()
* Call MoveArm()

Author: Matthew
http://projectsfromtech.blogspot.com/
9/2/2013

*/


#include <Servo.h>

Servo ServoS_1;      // Joint at base
Servo ServoS_2;      // Elbow between a and b

// Servo Angles
float ServoS_1_Angle = 90;
float ServoS_2_Angle = 90;

// Define arm Constants
const float a = 10.4;      // lower joint length (cm)
const float b = 10.4;      // upper joint length (cm)

// Correction factors to align servo values with their respective axis
const float S_1_CorrectionFactor = -10;     // Align arm "a" with the horizontal when at 0 degrees
const float S_2_CorrectionFactor = -77;     // Align arm "b" with arm "a" when at 0 degrees

// Correction factor to shift origin out to edge of the mount
const float X_CorrectionFactor = 0;       // X direction correction factor (cm)
const float Y_CorrectionFactor = 0;       // Y direction correction factor (cm)

// Angle Variables
float A;            //Angle oppposite side a (between b and c)
float B;            //Angle oppposite side b
float C;            //Angle oppposite side c
float theta;        //Angle formed between line from origin to (x,y) and the horizontal

// Distance variables
float x;            // x position (cm)
float y;            // y position (cm)
float c;            // Hypotenuse legngth in cm
const float pi = M_PI;  //Store pi in a less annoying format

//===================================================================================

void setup()
{
  ServoS_1.attach(2);             // Attach servos
  ServoS_2.attach(4);
  Serial.begin(9600);             // -For debugging
}
//--------------------------------------------------------------


double xdata[36] = {0.000000, 0.173643, 0.342010, 0.499987, 0.642772, 0.766028, 0.866010, 0.939680, 0.984801, 1.000000, 0.984817, 0.939712, 0.866056, 0.766087, 0.642843, 0.500067, 0.342098, 0.173734, 0.000093, -0.173552, -0.341923, -0.499906, -0.642701, -0.765968, -0.865964, -0.939649, -0.984785, -1.000000, -0.984833, -0.939744, -0.866103, -0.766147, -0.642914, -0.500147, -0.342185, -0.173826};
double ydata[36] = {1.000000, 0.984809, 0.939696, 0.866033, 0.766058, 0.642807, 0.500027, 0.342054, 0.173689, 0.000046, -0.173597, -0.341967, -0.499947, -0.642736, -0.765998, -0.865987, -0.939664, -0.984793, -1.000000, -0.984825, -0.939728, -0.866079, -0.766117, -0.642878, -0.500107, -0.342141, -0.173780, -0.000139, 0.173506, 0.341880, 0.499866, 0.642665, 0.765939, 0.865940, 0.939633, 0.984776};
void loop()
{
  int i;
  int circle_radius = 20;
  for (i = 0; i < 36; i++)
  {
    FixCoordinates(xdata[i] * circle_radius, ydata[i] * circle_radius);           // Enter coordinates of point.
    CalculateServoAngles();           // Calculate necessary angles of servos
    MoveArm();                        // Move arm to new position
    delay(1);
  }
}

//====================================================================================

// Get x and y measured from the bottom of the base. Function corrects for offset
void FixCoordinates(float x_input, float y_input)
{
 x = x_input + X_CorrectionFactor;
 y = y_input + Y_CorrectionFactor;
}

// Calculate necessary servo angles to move arm to desired points
void CalculateServoAngles()
{
  c = sqrt( sq(x) + sq(y) );                                            // pythagorean theorem
  B = (acos( (sq(b) - sq(a) - sq(c))/(-2*a*c) )) * (180/pi);            // Law of cosines: Angle opposite upper arm section
  C = (acos( (sq(c) - sq(b) - sq(a))/(-2*a*b) )) * (180/pi);            // Law of cosines: Angle opposite hypotenuse
  theta = (asin( y / c )) * (180/pi);                                   // Solve for theta to correct for lower joint's impact on upper joint's angle
  ServoS_1_Angle = B + theta + S_1_CorrectionFactor;                    // Find necessary angle. Add Correction
  ServoS_2_Angle = C + S_2_CorrectionFactor;                            // Find neceesary angle. Add Correction

}

// Update the servos
void MoveArm()
{
  ServoS_1.write(ServoS_1_Angle);              // Move joint to desired position
  ServoS_2.write(ServoS_2_Angle);              // Move joint to desired position
}
