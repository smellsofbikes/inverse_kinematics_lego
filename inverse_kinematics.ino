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

double xdata[36] = {0.000000, 3.472862, 6.840209, 9.999733, 12.855437, 15.320558, 17.320199, 18.793606, 19.696012, 20.000000, 19.696334, 18.794240, 17.321126, 15.321749, 12.856856, 10.001337, 6.841951, 3.474687, 0.001853, -3.471037, -6.838468, -9.998128, -12.854017, -15.319367, -17.319273, -18.792972, -19.695690, -20.000000, -19.696655, -18.794873, -17.322052, -15.322940, -12.858276, -10.002942, -6.843692, -3.476512};
double ydata[36] = {20.000000, 19.696173, 18.793923, 17.320662, 15.321154, 12.856147, 10.000535, 6.841080, 3.473775, 0.000927, -3.471950, -6.839339, -9.998930, -12.854727, -15.319962, -17.319736, -18.793289, -19.695851, -20.000000, -19.696495, -18.794557, -17.321589, -15.322345, -12.857566, -10.002140, -6.842821, -3.475600, -0.002780, 3.470125, 6.837597, 9.997325, 12.853307, 15.318771, 17.318809, 18.792655, 19.695529};

void loop()
{
  int i;
  for (i = 0; i < 36; i++)
  {
    FixCoordinates(xdata[i], ydata[i]);           // Enter coordinates of point.
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
