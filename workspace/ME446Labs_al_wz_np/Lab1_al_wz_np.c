#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
// If calibrated correctly, the offsets will ensure that the motor thetas read as zero when the robot is in the zero position
float offset_Enc2_rad = -0.42; // -0.37;
float offset_Enc3_rad = 0.23; // 0.27;


// Your global variables.

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];

long arrayindex = 0;
int UARTprint = 0;

float random = 3.14;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

// DH thetas
float Th2DH = 0;
float Th3DH = 0;

// IK thetas
float IKTh1 = 0;
float IKTh2 = 0;
float IKTh3 = 0;

float IKtheta1motor = 0;
float IKtheta2motor = 0;
float IKtheta3motor = 0;

// position variables
float x = 0;
float y = 0;
float z = 0;


void mains_code(void);

//
// Main
//
void main(void)
{
	mains_code();
}




// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {

    // Change the value of the taus to drive the motors and move the arm
    // Positive torques correspond to the positive joint axes
    *tau1 = 0;
    *tau2 = 0;
    *tau3 = 0;

    //Motor torque limitation(Max: 5 Min: -5)

    // save past states
    if ((mycount%50)==0) {

        theta1array[arrayindex] = theta1motor;
        theta2array[arrayindex] = theta2motor;

        if (arrayindex >= 99) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }

    }

    if ((mycount%500)==0) {
        UARTprint = 1;
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }


    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;

    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
    Simulink_PlotVar4 = 0;

    // Homogenous Transformation matrix H03 for forward kinematics
    //    float H03[][] = {{cos(theta3motor)*cos(theta1motor), -sin(theta3motor)*cos(theta1motor), -sin(theta1motor), 0.254*cos(theta1motor)*(cos(theta3motor)+sin(theta2motor))},
    //                     {cos(theta3motor)*sin(theta1motor), -sin(theta3motor)*sin(theta1motor), cos(theta1motor), 0.254*sin(theta1motor)*(cos(theta3motor)+sin(theta2motor))},
    //                     {-sin(theta3motor), -cost(theta3motor), 0 , 0.254*cos(theta2motor)-0.254*sin(theta3motor)+0.254},
    //                     {0,0,0,1}};

    // Forward Kinematics, we only need the final positional equations from the HTM H03, as they give the final location of the EE
    x = 0.254*cos(theta1motor)*(cos(theta3motor)+sin(theta2motor));
    y = 0.254*sin(theta1motor)*(cos(theta3motor)+sin(theta2motor));
    z = 0.254*cos(theta2motor)-0.254*sin(theta3motor)+0.254;

    // Conversion from motor thetas to DH thetas based on the solved system of equations
    // Note that, for theta 1, the motor theta and the DH theta are the same
    Th2DH = theta2motor - PI/2;
    Th3DH = -theta2motor + theta3motor + PI/2;

    // Inverse Kinematics, solutions came from the geometry of the CRS robot arm
    // See lab report for more detail on how these were solved for
    IKTh1 = atan2(y,x);
    IKTh2 = atan2((0.254-z),sqrt(pow(x,2) + pow(y,2)))-acos( (pow(y,2)+pow(x,2)+pow((0.254-z),2)) / (2*(sqrt(pow(x,2)+pow(y,2)+pow((0.254-z),2))*(0.254))));
    IKTh3 = PI - acos( (-(pow((0.254-z),2)+pow(x,2)+pow(y,2)) + pow((0.254),2) + pow((0.254),2)) / (2*.254*.254) );

    // Conversion from DH thetas derived from IK to the motor thetas
    IKtheta1motor = IKTh1;
    IKtheta2motor = IKTh2 + PI/2;
    IKtheta3motor = IKTh3 + IKtheta2motor - PI/2;

    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
//        serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);
        serial_printf(&SerialA, "Motor Thetas: %.2f %.2f %.2f \n\r", printtheta1motor*180/PI, printtheta2motor*180/PI, printtheta3motor*180/PI);
        serial_printf(&SerialA, "Position: %.2f %.2f %.2f  \n\r", x, y, z);
        serial_printf(&SerialA, "IK DH Thetas: %.2f %.2f %.2f   \n\r", IKTh1*180/PI, IKTh2*180/PI, IKTh3*180/PI);
        serial_printf(&SerialA, "Motor Theta (IK)s: %.2f %.2f %.2f  \n\r", IKtheta1motor*180/PI, IKtheta2motor*180/PI, IKtheta3motor*180/PI );
        serial_printf(&SerialA, "----------------\n\r");
    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }
}

