
//
// Version : 4
//
//

//
// heavily dependent:
// http://web.mit.edu/~jinstone/Public/filter.pdf
//


#include <math.h>
#include "cc.h"


#define PI                              3.1415926f
#define HALF_PI                         1.5707963f
#define TWO_PI                          6.2831853f
#define SQRE(x) 		                ((x)*(x))


CompSixAxis::
CompSixAxis(float deltaTime, float tau)
{
    // Save value to class
    deltaT = deltaTime;

    // Calculate weighting factor
    alpha = tau/(tau + deltaT);

    // Initialize other class variables
    compAngleX = 0;
    compAngleY = 0;
    accelAngleX = 0;
    accelAngleY = 0;
    Ax = 0;
    Ay = 0;
    Az = 0;
    Gx = 0;
    Gy = 0;
    Gz = 0;
}

void CompSixAxis::
CompStart()
{
    // Calculate accelerometer angles
    CompAccelCalculate();

    // Initialize filter to accel angles
    compAngleX = accelAngleX;
    compAngleY = accelAngleY;
}

void CompSixAxis::
CompUpdate()
{
    // Calculate the accelerometer angles
    CompAccelCalculate();


    compAngleX = CompFilterProcess(compAngleX, accelAngleX, -Gy);
   compAngleY = CompFilterProcess(compAngleY, accelAngleY, Gx);
}

void CompSixAxis::
CompAnglesGet(float *XAngle, float *YAngle)
{

    // Check if valid addresses were passed as well.
    if(XAngle)
    {
        *XAngle = compAngleX;
    }
    if(YAngle)
    {
        *YAngle = compAngleY;
    }
}

void CompSixAxis::
CompAccelUpdate(float accelX, float accelY, float accelZ)
{
    // Save values to class
    Ax = accelX;
    Ay = accelY;
    Az = accelZ;
}

void CompSixAxis::
CompGyroUpdate(float gyroX, float gyroY, float gyroZ)
{
    // Save values to class
    Gx = gyroX;
    Gy = gyroY;
    Gz = gyroZ;
}


void CompSixAxis::
CompAccelCalculate()
{
    // Angle made by X axis acceleration vector relative to ground
    accelAngleX = atan2f(Ax, sqrtf( SQRE(Ay) + SQRE(Az) ) );

    // Angle made by Y axis acceleration vector relative to ground
    accelAngleY = atan2f(Ay, sqrtf( SQRE(Ax) + SQRE(Az) ) );

    // Format the accel. angles to lie in the range of 0 to 2*pi
    accelAngleX = FormatAccelRange(accelAngleX, Az);
    accelAngleY = FormatAccelRange(accelAngleY, Az);
}

//
// Check to see which quadrant of the unit circle the angle lies in
// and format the angle to lie in the range of 0 to 2*PI
//
float CompSixAxis::
FormatAccelRange(float accelAngle, float accelZ)
{
    if(accelZ < 0.0f)
    {
        // Angle lies in Quadrant 2 or Quadrant 3 of
        // the unit circle
        accelAngle = PI - accelAngle;
    }
    else if(accelZ > 0.0f && accelAngle < 0.0f)
    {
        // Angle lies in Quadrant 4 of the unit circle
        accelAngle = TWO_PI + accelAngle;
    }

    // If both of the previous conditions were not satisfied, then
    // the angle must lie in Quadrant 1 and nothing more needs
    // to be done.

    return accelAngle;
}

//
// Formats the complimentary filter angle for faster convergence of the filter.
//
float CompSixAxis::
FormatFastConverge(float compAngle, float accAngle)
{
    // Work with comp. angles that are closest in distance to the accelerometer angle
    // on the unit circle. This allows for significantly faster filter convergence.
    if(compAngle > accAngle + PI)
    {
        compAngle = compAngle - TWO_PI;
    }
    else if(accAngle > compAngle + PI)
    {
        compAngle = compAngle + TWO_PI;
    }

    return compAngle;
}

//
// Formats the complimentary filter angle to always lie within the range of
// 0 to 2*pi
//
float CompSixAxis::
FormatRange0to2PI(float compAngle)
{
    while(compAngle >= TWO_PI)
    {
        compAngle = compAngle - TWO_PI;
    }

    while(compAngle < 0.0f)
    {
        compAngle = compAngle + TWO_PI;
    }

    return compAngle;
}

//
// Complimentary Filter logic
//
float CompSixAxis::
CompFilterProcess(float compAngle, float accelAngle, float omega)
{
    float gyroAngle;

    // Speed up filter convergence
    compAngle = FormatFastConverge(compAngle, accelAngle);


    gyroAngle = compAngle + omega*deltaT;


    compAngle = alpha*gyroAngle + (1.0f - alpha)*accelAngle;


    compAngle = FormatRange0to2PI(compAngle);

    return compAngle;
}
