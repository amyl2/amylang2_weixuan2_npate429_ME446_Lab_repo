#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
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

// PD controller constants
float Kp1 = 20.0;
float Kp2 = 75;
float Kp3 = 65;
float Kd1 = 1.35;
float Kd2 = 2.35;
float Kd3 = 2.0;

// PID controller constants
float thresh = 0.03;

float Kpt1 = 150;
float Kpt2 = 300;
float Kpt3 = 250;
float Kdt1 = 2.1;
float Kdt2 = 8;
float Kdt3 = 4;
float Ki1 = 175;
float Ki2 = 1000;
float Ki3 = 2000;

// Other variables for PD control
float theta_desired = 0.0;
float Theta1_old = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;

float Theta2_old = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;

float Theta3_old = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;

// PID controller variables
float Ik1 = 0;
float Ik2 = 0;
float Ik3 = 0;
float Ik1_old = 0;
float Ik2_old = 0;
float Ik3_old = 0;
float error1 = 0;
float error2 = 0;
float error3 = 0;
float error_old1 = 0;
float error_old2 = 0;
float error_old3 = 0;

// Feed Forward variables
float t = 0.0;
float J1 = 0.0167;
float J2 = 0.03;
float J3 = 0.0128;
float theta_desired_dot = 0.0;
float theta_desired_ddot = 0.0;

// Wh are there more?
float theta_desired1 = 0.0;
float theta_desired2 = 0.0;
float theta_desired3 = 0.0;

// Functions
float* trajectory(float t);

float* fun_trajectory(float t);

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

//    *tau1 = 0;
//    *tau2 = 0;
//    *tau3 = 0;

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

//    // Desired theta square wave generation
//    if ((mycount%2000) < 1000) {
//        theta_desired = PI/6;
//    } else {
//        theta_desired = 0;
//    }

//    float* trajectory_array = trajectory((mycount%3000)/1000.0);
//    theta_desired = trajectory_array[0];
//    theta_desired_dot = trajectory_array[1];
//    theta_desired_ddot = trajectory_array[2];
//    free(trajectory_array);

    float* trajectory_fun = fun_trajectory((mycount%3000)/1000.0);
    theta_desired1 = trajectory_fun[0];
    theta_desired2 = trajectory_fun[1];
    theta_desired3 = trajectory_fun[2];
    free(trajectory_fun);

    // IIR
    Omega1 = (theta1motor - Theta1_old)/0.001;
    Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0;
    Theta1_old = theta1motor;
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;

    Omega2 = (theta2motor - Theta2_old)/0.001;
    Omega2 = (Omega2 + Omega2_old1 + Omega2_old2)/3.0;
    Theta2_old = theta2motor;
    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;

    Omega3 = (theta3motor - Theta3_old)/0.001;
    Omega3 = (Omega3 + Omega3_old1 + Omega3_old2)/3.0;
    Theta3_old = theta3motor;
    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;


    if (fabs(theta_desired - theta1motor) > thresh) {
        // PD controller
        Ik1 = 0;
        Ik1_old = 0;
//       *tau1 = Kp1*(theta_desired-theta1motor)-Kd1*Omega1;
//        *tau1 = J1*theta_desired_ddot + Kp1*(theta_desired-theta1motor)+Kd1*(theta_desired_dot-Omega1);
        *tau1 = Kp1*(theta_desired1-theta1motor)-Kd1*Omega1;

    } else {
        // PID controller
        error1 = theta_desired - theta1motor;
        Ik1_old = Ik1;
        Ik1 = Ik1_old + (error1 + error_old1)/2 * 0.001;
        error_old1 = error1;
//        *tau1 = Kpt1*(theta_desired-theta1motor)-Kdt1*Omega1 + Ki1*Ik1;
//        *tau1 = J1*theta_desired_ddot+ Kpt1*(theta_desired-theta1motor)+Kdt1*(theta_desired_dot-Omega1) + Ki1*Ik1;
        *tau1 = Kpt1*(theta_desired1-theta1motor)-Kdt1*Omega1 + Ki1*Ik1;

    }

    if (fabs(theta_desired - theta2motor) > thresh) {
        // PD controller
        Ik2 = 0;
        Ik2_old = 0;
//        *tau2 = Kp2*(theta_desired-theta2motor)-Kd2*Omega2;
//        *tau2 = J2*theta_desired_ddot + Kp2*(theta_desired-theta2motor)+Kd2*(theta_desired_dot-Omega2);
        *tau2 = Kp2*(theta_desired2-theta2motor)-Kd2*Omega2;
    } else {
        // PID controller
        error2 = theta_desired - theta2motor;
        Ik2_old = Ik2;
        Ik2 = Ik2_old + (error2 + error_old2)/2 * 0.001;
        error_old2 = error2;
//        *tau2 = Kpt2*(theta_desired-theta2motor)-Kdt2*Omega2 + Ki2*Ik2;
//        *tau2 = J2*theta_desired_ddot+ Kpt2*(theta_desired-theta2motor)+Kdt2*(theta_desired_dot-Omega2) + Ki2*Ik2;
        *tau2 = Kpt2*(theta_desired2-theta2motor)-Kdt2*Omega2 + Ki2*Ik2;
    }

    if (fabs(theta_desired - theta3motor) > thresh) {
        // PD controller
        Ik3 = 0;
        Ik3_old = 0;
//        *tau3 = Kp3*(theta_desired-theta3motor)-Kd3*Omega3;
//        *tau3 = J3*theta_desired_ddot + Kp3*(theta_desired-theta3motor)+Kd3*(theta_desired_dot-Omega3);
        *tau3 = Kp3*(theta_desired3-theta3motor)-Kd3*Omega3;
    } else {
        // PID controller
        error3 = theta_desired - theta3motor;
//        Ik3 = Ik3 + (error3 + error_old3)/2 * 0.001;
        Ik3_old = Ik3;
        Ik3 = Ik3_old + (error3 + error_old3)/2 * 0.001;
        error_old3 = error3;
//        *tau3 = Kpt3*(theta_desired-theta3motor)-Kdt3*Omega3 + Ki3*Ik3;
//        *tau3 = J3*theta_desired_ddot+ Kpt3*(theta_desired-theta3motor)+Kdt3*(theta_desired_dot-Omega3) + Ki3*Ik3;
        *tau3 = Kpt3*(theta_desired3-theta3motor)-Kdt3*Omega3 + Ki3*Ik3;
    }



//     Saturate the Tau values
    if (*tau1 > 5.0){
        *tau1 = 5.0;
        Ik1 = Ik1_old;
    } else if ( *tau1 < -5.0 ){
        *tau1 = -5.0;
        Ik1 = Ik1_old;
    }

    if (*tau2 > 5.0){
        *tau2 = 5.0;
        Ik2 = Ik2_old;
    } else if ( *tau2 < -5.0 ){
        *tau2 = -5.0;
        Ik2 = Ik2_old;
    }

    if (*tau3 > 5.0){
        *tau3 = 5.0;
        Ik3 = Ik3_old;
    } else if ( *tau3 < -5.0 ){
        *tau3 = -5.0;
        Ik3 = Ik3_old;
    }


    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;

    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
    Simulink_PlotVar4 = theta_desired;

    // Plot erros
//    Simulink_PlotVar1 = theta_desired-theta1motor;
//    Simulink_PlotVar2 = theta_desired-theta2motor;
//    Simulink_PlotVar3 = theta_desired-theta3motor;
//    Simulink_PlotVar4 = theta_desired-theta_desired;

    // Homogenous Transformation matrix H03 for forward kinematics
    //    float H03[][] = {{cos(theta3motor)*cos(theta1motor), -sin(theta3motor)*cos(theta1motor), -sin(theta1motor), 0.254*cos(theta1motor)*(cos(theta3motor)+sin(theta2motor))},
    //                     {cos(theta3motor)*sin(theta1motor), -sin(theta3motor)*sin(theta1motor), cos(theta1motor), 0.254*sin(theta1motor)*(cos(theta3motor)+sin(theta2motor))},
    //                     {-sin(theta3motor), -cost(theta3motor), 0 , 0.254*cos(theta2motor)-0.254*sin(theta3motor)+0.254},
    //                     {0,0,0,1}};

    // Forward Kinematics
    x = 0.254*cos(theta1motor)*(cos(theta3motor)+sin(theta2motor));
    y = 0.254*sin(theta1motor)*(cos(theta3motor)+sin(theta2motor));
    z = 0.254*cos(theta2motor)-0.254*sin(theta3motor)+0.254;

    Th2DH = theta2motor - PI/2;
    Th3DH = -theta2motor + theta3motor + PI/2;

    // Inverse Kinematics
    IKTh1 = atan2(y,x);
    IKTh2 = atan2((0.254-z),sqrt(pow(x,2) + pow(y,2)))-acos( (pow(y,2)+pow(x,2)+pow((0.254-z),2)) / (2*(sqrt(pow(x,2)+pow(y,2)+pow((0.254-z),2))*(0.254))));
    IKTh3 = PI - acos( (-(pow((0.254-z),2)+pow(x,2)+pow(y,2)) + pow((0.254),2) + pow((0.254),2)) / (2*.254*.254) );

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

float* trajectory(float t) {
    float* res = malloc(3.0 * sizeof(float));
    if (t>= 0 && t<=1) {
        theta_desired = -pow(t,3) +1.5*pow(t,2);
        theta_desired_dot = -3*pow(t,2)+3.0*t;
        theta_desired_ddot = 3 - 6*t;
    }
    else if (t > 1 && t<=2 ) {
        theta_desired = pow(t,3)-4.5*pow(t,2)+6*t-2;
        theta_desired_dot = 3*pow(t,2)-9*t+6;
        theta_desired_ddot = 6*t-9;
    }
    else {
        theta_desired = 0.0;
        theta_desired_dot = 0.0;
        theta_desired_ddot = 0.0;
    }
    *res = theta_desired;
    *(res + 1) = theta_desired_dot;
    *(res + 2) = theta_desired_ddot;
    return res;
}

float* fun_trajectory(float t){
    float* res = malloc(3 * sizeof(float));

    float t0 = 0;
    float t02 = 1;
    float t_total = 1;
    if (t>= 0 && t<=1) {
        float x = 0.25;
        float y = 0;
        float z1 = (0.50-0.35)*((t-t0)/t_total) + 0.35;
        theta_desired1 = atan2(y,x);
        theta_desired2 = atan2((0.254-z),sqrt(pow(x,2) + pow(y,2)))-acos( (pow(y,2)+pow(x,2)+pow((0.254-z),2)) / (2*(sqrt(pow(x,2)+pow(y,2)+pow((0.254-z),2))*(0.254))));
        theta_desired3 = PI - acos( (-(pow((0.254-z),2)+pow(x,2)+pow(y,2)) + pow((0.254),2) + pow((0.254),2)) / (2*.254*.254) );
        }
    else if (t > 1 && t<=2 ) {
        float x = 0.25;
        float y = 0;
        float z2 = (0.35-0.5)*((t-t02)/t_total) + 0.5;
        theta_desired1 = atan2(y,x);
        theta_desired2 = atan2((0.254-z),sqrt(pow(x,2) + pow(y,2)))-acos( (pow(y,2)+pow(x,2)+pow((0.254-z),2)) / (2*(sqrt(pow(x,2)+pow(y,2)+pow((0.254-z),2))*(0.254))));
        theta_desired3 = PI - acos( (-(pow((0.254-z),2)+pow(x,2)+pow(y,2)) + pow((0.254),2) + pow((0.254),2)) / (2*.254*.254) );
    }
    *res = theta_desired1;
    *(res + 1) = theta_desired2;
    *(res + 2) = theta_desired3;
    return res;
}
