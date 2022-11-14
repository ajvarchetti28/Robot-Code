#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHBattery.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <math.h>
#include <iostream>
#include <FEHSD.h>
#include <FEHSD.h>
#include <FEHServo.h>
#include <FEHAccel.h>
#define PI 3.14159265359
using namespace std;
//--------Fuction Prototypes-------
void resetPIDVariables();
void calculateMotorSpeedExpected(float[], float[]);
float inchesToMotorPercent(float, int);
void setSpeed(float, float, float);
void check_heading(float);
void turn_counterclockwise(float, float);
void turnTo(float);
void moveTo(float, float, float);
void convergeOn(float, float, float, float);
void worldToRobot(float[], float[]);
void robotToWorld(float[], float[]);
void robotToMotor(float[], float[]);
void motorToRobot(float[], float[]);
void worldRobotPID(float[], float[]);
void robotPID(float[], float[]);
void motorPID(float[], float[]);
float getMotorSpeed(float, int);
//--------Global Variables---------
FEHMotor mot1(FEHMotor::Motor0, 9.0);
FEHMotor mot2(FEHMotor::Motor1, 9.0);
FEHMotor mot3(FEHMotor::Motor2, 9.0);
DigitalEncoder mot1_encoder(FEHIO::P0_0);
DigitalEncoder mot2_encoder(FEHIO::P1_0);
DigitalEncoder mot3_encoder(FEHIO::P2_0);
FEHServo servoArm(FEHServo::Servo1);
AnalogInputPin cds(FEHIO::P3_0);
float motorPreviousSpeed[] = {0, 0, 0};
bool isBlue = false;
bool needSleep = false;
float sleep = 0.0;
// Contants
float R = 3 + 7 / 16.0;
float r = 1.25;
float servoMax = 1650;
float servoMin = 680;
//--------------
// Classes
//--------------
class MotorPID
{
public:
    MotorPID(int _number, double _max, double _min, double _Kp, double _Kd,
             double _Ki, double _previousError, double _sumOfErrors, double _previousTime,
             double previousCounts);
    double calculateAdjustment(double expected, double actual, FEHFile *fptr);
    void resetPIDVariables();

private:
    int number;
    double max;
    double min;
    double Kp;
    double Kd;
    double Ki;
    double previousError;
    double sumOfErrors;
    double previousTime;
    double previousCounts;
};
//! Function protypes here
void drivePID(float, float, float, MotorPID, MotorPID, MotorPID, FEHFile *);
int main(void)
{
    // Jukebox Light RPS Values
    float jLightX = 0;
    float jLightY = 0;
    // BurgerTask RPS Values
    float burgerX = 0;
    float burgerY = 0;
    float burgerHeading = 0;
    // Tray
    float trayX = 0;
    float trayY = 0;
    // Vanilla
    float vanillaX = 0;
    float vanillaY = 0;
    float vanillaHeading = 0;
    // Twist
    float twistX = 0;
    float twistY = 0;
    float twistHeading = 0;
    // Chocolate
    float chocolateX = 0;
    float chocolateY = 0;
    float chocolateHeading = 0;
    // ticket
    float ticketX = 0;
    float ticketY = 0;
    servoArm.SetDegree(15.0); // 15
    RPS.InitializeTouchMenu();
    float x, y;
    FEHFile *fptr = SD.FOpen("data.txt", "w");
    // Set servo min and max
    servoArm.SetMax(servoMax);
    servoArm.SetMin(servoMin);
    int lever = RPS.GetIceCream();
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);
    LCD.Write("Battery Voltage: ");
    LCD.WriteLine(Battery.Voltage());
    LCD.WriteLine("Waitng for RPS Values of light...");
    while (!LCD.Touch(&x, &y))
    {
        LCD.Write("cds val: ");
        LCD.WriteLine(cds.Value());
        Sleep(0.5);
        // LCD.WriteLine("");
        // LCD.Write("X: ");
        // LCD.WriteLine(RPS.X());
        // LCD.Write("Y: ");
        // LCD.WriteLine(RPS.Y());
        // LCD.Write("Heading: ");
        // Sleep(0.5);
    }
    Sleep(0.1);
    jLightX = RPS.X();
    jLightY = RPS.Y();
    LCD.Clear(BLACK);
    Sleep(1.5);
    LCD.WriteLine("Waitng for RPS Values of tray...");
    while (!LCD.Touch(&x, &y))
    {
        LCD.WriteLine("");
        LCD.Write("X: ");
        LCD.WriteLine(RPS.X());
        LCD.Write("Y: ");
        LCD.WriteLine(RPS.Y());
        LCD.Write("Heading: ");
        Sleep(0.5);
    }
    Sleep(0.1);
    trayX = RPS.X();
    trayY = RPS.Y();
    LCD.Clear(BLACK);
    Sleep(1.5);
    servoArm.SetDegree(180);
    Sleep(0.5);
    LCD.WriteLine("Waitng for RPS Values of Burger...");
    while (!LCD.Touch(&x, &y))
    {
        LCD.WriteLine("");
        LCD.Write("X: ");
        LCD.WriteLine(RPS.X());
        LCD.Write("Y: ");
        LCD.WriteLine(RPS.Y());
        LCD.Write("Heading: ");
        LCD.WriteLine(RPS.Heading());
        Sleep(0.5);
    }
    Sleep(0.1);
    burgerX = RPS.X() - 1.0; // Callibration
    burgerY = RPS.Y() - 3.0; // Callibration
    burgerHeading = RPS.Heading();
    LCD.Clear(BLACK);
    Sleep(0.5);
    LCD.WriteLine("Waitng for RPS Values of Vanilla...");
    while (!LCD.Touch(&x, &y))
    {
        LCD.WriteLine("");
        LCD.Write("X: ");
        LCD.WriteLine(RPS.X());
        LCD.Write("Y: ");
        LCD.WriteLine(RPS.Y());
        LCD.Write("Heading: ");
        LCD.WriteLine(RPS.Heading());
        Sleep(0.5);
    }
    Sleep(0.1);
    vanillaX = RPS.X();
    vanillaY = RPS.Y();
    vanillaHeading = RPS.Heading();
    LCD.Clear(BLACK);
    Sleep(0.5);
    LCD.WriteLine("Waitng for RPS Values of Twist...");
    while (!LCD.Touch(&x, &y))
    {
        LCD.WriteLine("");
        LCD.Write("X: ");
        LCD.WriteLine(RPS.X());
        LCD.Write("Y: ");
        LCD.WriteLine(RPS.Y());
        LCD.Write("Heading: ");
        LCD.WriteLine(RPS.Heading());
        Sleep(0.5);
    }
    Sleep(0.1);
    twistX = RPS.X();
    twistY = RPS.Y();
    twistHeading = RPS.Heading();
    LCD.Clear(BLACK);
    Sleep(0.5);
    LCD.WriteLine("Waitng for RPS Values of Chocolate...");
    while (!LCD.Touch(&x, &y))
    {
        LCD.WriteLine("");
        LCD.Write("X: ");
        LCD.WriteLine(RPS.X());
        LCD.Write("Y: ");
        LCD.WriteLine(RPS.Y());
        LCD.Write("Heading: ");
        LCD.WriteLine(RPS.Heading());
        Sleep(0.5);
    }
    Sleep(0.1);
    chocolateX = RPS.X();
    chocolateY = RPS.Y();
    chocolateHeading = RPS.Heading();
    LCD.Clear(BLACK);
    Sleep(0.5);
    LCD.WriteLine("Waitng for RPS Values of Ticket...");
    while (!LCD.Touch(&x, &y))
    {
        LCD.WriteLine("");
        LCD.Write("X: ");
        LCD.WriteLine(RPS.X());
        LCD.Write("Y: ");
        LCD.WriteLine(RPS.Y());
        LCD.Write("Heading: ");
        LCD.WriteLine(RPS.Heading());
        Sleep(0.5);
    }
    Sleep(0.1);
    ticketX = RPS.X();
    ticketY = RPS.Y() + 3.5;
    LCD.Clear(BLACK);
    Sleep(1.5);
    servoArm.SetDegree(0.0);
    Sleep(1.0);
    LCD.WriteLine("Waiting For Starting Light...");
    while (!LCD.Touch(&x, &y))
    {
    };
    float initialCDS = cds.Value();
    while (cds.Value() > .8 * initialCDS)
    {
        Sleep(0.5);
    }
    float initialRedAvg = 0;
    for (int i = 0; i < 5; i++)
    {
        initialRedAvg += cds.Value();
    }
    initialRedAvg /= 5;
    // Wait for Starting light
    LCD.WriteLine("Start!");
    LCD.WriteLine("Compiled once again");
    // //Performance Test 1: Jukebox -> Up the ramp
    // Moves towards Jukebox Light
    setSpeed(0, -12, 0);
    float currentCDS = cds.Value();
    Sleep(0.3);
    convergeOn(jLightX, jLightY, .4, .4);
    float cdsAvg = 0;
    for (int i = 0; i < 5; i++)
    {
        cdsAvg += cds.Value();
    }
    cdsAvg /= 5;
    // If Red
    if (abs(cdsAvg - initialRedAvg) < .15)
    {
        LCD.WriteLine("Red");
        isBlue = false;
        moveTo(3, 0, 15);
    }
    // If Blue(default)
    else
    {
        LCD.WriteLine("Blue");
        isBlue = true;
        moveTo(20, 1, 15);
    }
    // Move to middle
    convergeOn(trayX, 20, .3, 1); // was 18.0
    Sleep(0.01);
    if (abs(RPS.Heading() - 270) > 3)
    {
        turnTo(180);
        Sleep(0.1);
        turnTo(270);
        Sleep(.1);
    }
    // Move up Ramp
    setSpeed(0, 0, 24);
    Sleep(2.0);
    setSpeed(0, 0, 0);
    // Go to point above ramp
    convergeOn(trayX, trayY, .5, .5);
    Sleep(.1); // 0.3
    // Throw tray in sink
    turnTo(132.5);
    LCD.WriteLine("KOBE!!!");
    Sleep(.25); //.5
    servoArm.SetDegree(120.0);
    Sleep(.2);
    servoArm.SetDegree(15.0);
    // Go to middle of ice cream levers
    // Turn towards correct lever
    if (lever == 2)
    {
        convergeOn(chocolateX, chocolateY, .4, .4);
        // Sleep(0.3);
        turnTo(chocolateHeading);
    }
    else if (lever == 1)
    {
        convergeOn(twistX, twistY, .4, .4);
        // Sleep(0.3);
        turnTo(twistHeading);
    }
    else
    {
        convergeOn(vanillaX, vanillaY, .4, .4);
        // Sleep(0.3);
        turnTo(vanillaHeading);
    }
    // Move towards lever
    Sleep(.1); // 0.3
    setSpeed(0, -10, 0);
    Sleep(1.25);
    setSpeed(0, 0, 0);
    Sleep(0.01); // 0.1
    // Push lever down
    servoArm.SetDegree(120);
    Sleep(0.4);
    servoArm.SetDegree(15);
    Sleep(.01); // 0.1
    // Move Back
    setSpeed(0, 10, 0);
    Sleep(1.3);
    setSpeed(0, 0, 0);
    // Go to Burger
    // TODO:Get RPS Value at the start
    LCD.WriteLine("Going to burger");
    convergeOn(burgerX, burgerY, 0.5, 1);
    setSpeed(0, 0, 0);
    turnTo(45.6);
    servoArm.SetDegree(180);
    turnTo(47.0);
    LCD.WriteLine("Ready to flip");
    // Drive a little forward to get under handle
    setSpeed(0, -10, -10);
    Sleep(0.4);
    setSpeed(0, 0, 0);
    // Drive a little right to get under handle
    setSpeed(0, 10, -10);
    Sleep(0.2);
    setSpeed(0, 0, 0);
    // Flip arm up and drive right
    servoArm.SetDegree(120);
    Sleep(0.3);
    setSpeed(0, 10, -10);
    Sleep(0.3);
    setSpeed(0, 0, 0);
    // Spin the robot to fling the burger tray up
    Sleep(0.2);
    mot1.SetPercent(-90.0);
    mot2.SetPercent(-90.0);
    mot3.SetPercent(-90.0);
    Sleep(0.1);
    mot1.SetPercent(90.0);
    mot2.SetPercent(90.0);
    mot3.SetPercent(90.0);
    Sleep(.01);
    servoArm.SetDegree(15);
    Sleep(0.15);
    setSpeed(0, 0, 0);
    Sleep(0.3);
    // Move back then spin robot to hit tray down
    //  setSpeed(0,10,8);
    //  Sleep(0.1);
    //  setSpeed(0,0,0);
    //  mot1.SetPercent(90.0);
    //  mot2.SetPercent(90.0);
    //  mot3.SetPercent(90.0);
    //  Sleep(1.0);
    //  setSpeed(0,0,0);
    //  Sleep(0.3);
    // Move to Ticket
    convergeOn(ticketX, ticketY, .25, .5);
    turnTo(225.0);
    servoArm.SetDegree(180);
    setSpeed(0, -8, -10);
    Sleep(0.65);
    setSpeed(0, 0, 0);
    // Sleep(0.3);
    // Move the ticket
    Sleep(0.05);
    setSpeed(0, 10, -9);
    Sleep(0.5);
    setSpeed(0, 0, 0);
    turnTo(150);
    Sleep(0.5);
    turnTo(250);
    Sleep(0.1); //.75
    setSpeed(0, 0, 0);
    Sleep(.20);
    setSpeed(0, 20, 0); // Move back out of ticket
    Sleep(0.5);
    setSpeed(0, 0, 0);
    servoArm.SetDegree(15);
    Sleep(0.5);
    // Go back to Ice Cream
    // Hit lever up
    if (lever == 2)
    {
        convergeOn(chocolateX - 1, chocolateY - 1, .4, .4);
        Sleep(0.3);
        turnTo(chocolateHeading);
        servoArm.SetDegree(160);
        Sleep(0.3);
    }
    else if (lever == 1)
    {
        convergeOn(twistX - 1, twistY - 1, .4, .4);
        Sleep(0.3);
        turnTo(twistHeading);
        servoArm.SetDegree(160);
        Sleep(0.3);
    }
    else
    {
        convergeOn(vanillaX - 1, vanillaY - 1, .4, .4);
        Sleep(0.3);
        turnTo(vanillaHeading);
        servoArm.SetDegree(160);
        Sleep(0.3);
    }
    // Move towards lever
    setSpeed(0, -10, 0);
    Sleep(1.05);
    // Move towards lever
    setSpeed(0, 0, 0);
    Sleep(0.3);
    // Push lever up
    servoArm.SetDegree(80.0);
    Sleep(0.4);
    setSpeed(0, 24, 0);
    Sleep(.45); //.75
    while (RPS.X() < 0 || RPS.Y() < 0)
    {
        setSpeed(0, 10, 0);
        Sleep(0.3);
    }
    servoArm.SetDegree(15.0);
    setSpeed(0, 0, 0);
    Sleep(0.05); // .1
    // Go to Middle
    convergeOn(17.5, 45.2, 1, 5);
    Sleep(.3);
    // Face down and move down ramp
    turnTo(90);
    Sleep(0.01);
    setSpeed(0, 0, 24);
    Sleep(1.1);
    setSpeed(0, 0, 0);
    // Move towards button
    Sleep(0.02);
    turnTo(130.0); // 135
    setSpeed(0, 0, 24.0);
    Sleep(2.0);
    setSpeed(0, 0, 0);
    //! Safety things
    while (true)
    {
        setSpeed(0, 10, 10);
        Sleep(0.5);
        moveTo(40, -4, 20);
    }
    return 0;
}
// Angular Speed, X speed, Y Speed
void robotToMotor(float robotVelocity[], float motorVelocity[])
{
    // Get robot velocities
    float w = robotVelocity[0];
    float x = robotVelocity[1];
    float y = robotVelocity[2];
    // Convert robot velocities to motor velocities
    // Doesnt go straight
    if (y == 0)
    {
        motorVelocity[0] = 1.015 * (1 / r) * (x - R * w);
        motorVelocity[1] = (1 / r) * (-R * w - x / 2 - sqrt(3) * y / 2);
        motorVelocity[2] = (1 / r) * (-R * w - x / 2 + sqrt(3) * y / 2);
    }
    else if (x == 0)
    {
        motorVelocity[0] = (1 / r) * (x - R * w);
        motorVelocity[1] = 1.015 * (1 / r) * (-R * w - x / 2 - sqrt(3) * y / 2);
        motorVelocity[2] = (1 / r) * (-R * w - x / 2 + sqrt(3) * y / 2);
    }
    else
    {
        motorVelocity[0] = (1 / r) * (x - R * w);
        motorVelocity[1] = (1 / r) * (-R * w - x / 2 - sqrt(3) * y / 2);
        motorVelocity[2] = (1 / r) * (-R * w - x / 2 + sqrt(3) * y / 2);
    }
}
// Takes the desired motor speed in inches per second and the motor number, and
returns their speed float inchesToMotorPercent(float motorSpeed, int motor)
{
    //! Multiply is faster than division
    // Set previous motor
    motorPreviousSpeed[motor - 1] = motorSpeed;
    bool negative = false;
    if (motorSpeed < 0)
    {
        negative = true;
    }
    if (motorSpeed < 0.01 && motorSpeed > -.01)
    {
        return 0;
    }
    float newSpeed = 0;
    switch (motor)
    {
    case 1:
        newSpeed = (abs(motorSpeed) + 1.1289) / .2784;
        break;
    case 2:
        newSpeed = (abs(motorSpeed) + 1.3052) / .2837;
        break;
    case 3:
        newSpeed = (abs(motorSpeed) + 1.1876) / .2791;
        break;
    default:
        break;
    }
    if (negative)
    {
        return newSpeed * -1;
    }
    return newSpeed;
}
// W is angular speed, x is horizontal speed, y is vertical speed
// All are in inches a second
void setSpeed(float w, float x, float y)
{
    // Calculate motor speeds need to move the robot the desired speed
    float robotVelocities[] = {w, x, y};
    float motorVelocities[] = {0, 0, 0};
    robotToMotor(robotVelocities, motorVelocities);
    // Set the motors speeds accordingly
    mot1.SetPercent(inchesToMotorPercent(motorVelocities[1], 1));
    mot2.SetPercent(inchesToMotorPercent(motorVelocities[2], 2));
    mot3.SetPercent(inchesToMotorPercent(motorVelocities[0], 3));
}
void check_heading(float heading)
{
    // You will need to fill out this one yourself and take into account
    // checking for proper RPS data and the edge conditions
    //(when you want the robot to go to 0 degrees or close to 0 degrees)
    /*
    SUGGESTED ALGORITHM:
    1. Check the current orientation of the QR code and the desired
    orientation of the QR code
    2. Check if the robot is within the desired threshold for the heading
    based on the orientation
    3. Pulse in the correct direction based on the orientation
    */
    while (heading > (RPS.Heading() + .4) || heading < (RPS.Heading() - .4))
    {
        float percentError = abs(heading - RPS.Heading()) / RPS.Heading();
        if (heading > RPS.Heading())
        {
            turn_counterclockwise(percentError, .2);
        }
        else if (heading < RPS.Heading())
        {
            turn_counterclockwise(-1 * percentError, .2);
        }
    }
    // JUST IN CASE IT DOESN'T SET BACK TO ZERO FOR SOME REASON:
    setSpeed(0, 0, 0);
    LCD.WriteLine("FINISHED HEADING ORIENTATION");
}
void turnTo(float degree)
{
    Sleep(.3);
    LCD.WriteLine("turn");
    float difference = RPS.Heading() - degree;
    // if (difference > 180.0) {
    // difference = difference - 360.0;
    // } else if (difference < -180.0) {
    // difference = difference + 360.0;
    // }
    if (difference < 0)
    { // Turn CounterClockwise
        LCD.WriteLine("CCW");
        setSpeed(-(36 / 35.0) * 7.8, 0, 0);
        Sleep(-1 * difference / 360.0);
    }
    else
    { // Turn Clockwise'=
        LCD.WriteLine("CW");
        setSpeed((36 / 35.0) * 7.8, 0, 0);
        Sleep(difference / 360.0);
    }
    setSpeed(0, 0, 0);
}
void moveTo(float x, float y, float speed)
{
    // Change in variables
    float changeX = x - RPS.X();
    float changeY = y - RPS.Y();
    turnTo(270.0);
    // Move robot at speed in/s
    float magnitude = sqrt(changeX * changeX + changeY * changeY);
    float multiplier = speed / magnitude;
    LCD.Write("x: ");
    LCD.WriteLine(-1 * multiplier * changeX);
    LCD.Write("y: ");
    LCD.WriteLine(multiplier * changeY);
    setSpeed(0, -1 * multiplier * changeX, multiplier * changeY);
    Sleep((magnitude / speed) * 1.05);
    setSpeed(0, 0, 0);
}
void convergeOn(float x, float y, float thresholdX, float thresholdY)
{
    while (abs(x - RPS.X()) > 3 && abs(y - RPS.Y()) > 3)
    {
        Sleep(0.15);
        moveTo(x, y, 20);
        Sleep(.3);
    }
    while (abs(x - RPS.X()) > 2 && abs(y - RPS.Y()) > 2)
    {
        moveTo(x, y, 15);
        Sleep(.3);
    }
    while (abs(x - RPS.X()) > thresholdX || abs(y - RPS.Y()) > thresholdY)
    {
        moveTo(x, y, 13);
        Sleep(.3);
    }
}
/*
* Turn counterclockwise using shaft encoders where percent is the motor percent
and counts is the distance to travel
*/
void turn_counterclockwise(float percent, float counts)
{
    // Reset encoder counts
    mot1_encoder.ResetCounts();
    mot2_encoder.ResetCounts();
    mot3_encoder.ResetCounts();
    if (percent < .5)
    {
        percent = .5;
    }
    // Set both motors to desired percent
    setSpeed(9 * percent, 0, 0);
    // While the average of the left and right encoder are less than counts,
    // keep running motors
    while ((mot1_encoder.Counts() + mot2_encoder.Counts() + mot3_encoder.Counts()) / 3.0 < counts)
        ;
    mot1_encoder.ResetCounts();
    mot2_encoder.ResetCounts();
    mot3_encoder.ResetCounts();
    // Turn off motors
    setSpeed(0, 0, 0);
}
