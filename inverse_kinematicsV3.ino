#include <stdio.h>
#include <math.h>
//#include <Servo.h>

//Servo arm1, arm2;

void ik_calc(double x, double y, double *angle1, double *angle2)
{
  double link1 = 103.67;
  double link2 = 103.67;
  double b, q1, q2;

  b = (x*x) + (y*y);
  q1 = atan2(y, x);
  q2 = acos(((link1 * link2) - (link2 * link2) + b)/(2*link1 * sqrt(b)));
  *angle1 = q1 + q2;
  *angle2 = acos(((link1 * link1) + (link2 * link2) - b)/(2*link1 * link2));
}

/*void setup()
{
  arm1.attach(9);
  arm2.attach(10);
}*/

int main(void)
{
  double x, y, rad_conv, theta1, theta2;
  int i;
  int xvals[] = {80, 100, 120};
  int yvals[] = {100, 100, 100};
  rad_conv = 57.296;
  int alen;
  alen = sizeof(xvals)/sizeof(xvals[0]);

  printf("x1_deg\t x1_rad\t x2_deg\t x2_rad\n");
  for (i = 0; i <alen ; i++)
  {
    x = xvals[i];
    y = yvals[i];
    ik_calc(x, y, &theta1, &theta2);


    printf("%f\t %f\t %f\t %f\n", (theta1 * rad_conv) - 50, theta1, 180 - (theta2 * rad_conv), theta2);
  }
}
