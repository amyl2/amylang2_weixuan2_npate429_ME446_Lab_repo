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

// Friction constants and parameters
float Viscous_positive1 = 0.195; // 0.2513 og
float Viscous_positive2 = 0.2500;
float Viscous_positive3 = 0.1922;
float Coulomb_positive1 = 0.3405; // 0.33487 og
float Coulomb_positive2 = 0.1675;
float Coulomb_positive3 = 0.17039;

float Viscous_negative1 = 0.195; // 0.2477 og
float Viscous_negative2 = 0.2870;
float Viscous_negative3 = 0.2132;
float Coulomb_negative1 = -0.3405; // -0.33523 og
float Coulomb_negative2 = -0.16565;
float Coulomb_negative3 = -0.16934;

float slope = 3.6;

float min_velocity1 = 0.1;
float min_velocity2 = 0.05;
float min_velocity3 = 0.05;

float u_fric1 = 0.0;
float u_fric2 = 0.0;
float u_fric3 = 0.0;

float fric_fac1 = 0.45;
float fric_fac2 = 1.0;
float fric_fac3 = 1.7;

//// Task Space constants
float KPx = 280.0;
float KPy = 150.0;
float KPz = 220.0;

float KDx = 19.0;
float KDy = 12.0;
float KDz = 12.0;

float xde = 0.35;
float yde = 0.0;
float zde = 0.40;

float xdd = 0.0;
float ydd = 0.0;
float zdd = 0.0;

float xd = 0.0;
float yd = 0.0;
float zd = 0.0;

float x_old = 0.0;
float xd_old1 = 0.0;
float xd_old2 = 0.0;
float y_old = 0.0;
float yd_old1 = 0.0;
float yd_old2 = 0.0;
float z_old = 0.0;
float zd_old1 = 0.0;
float zd_old2 = 0.0;

float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;
float thetaz = 0;
float thetax = 0;
float thetay = 0;
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;

// Feedforward force term
float Zcmd_offset = 10;
float Zcmd_force = 0;
float Zcmd = 0;
float Kt = 6.0;

// Impedance control (axis rotation)
float tx = 0.0;
float ty = 0.0;
float tz = PI/4;

float ctx = 0.0;
float cty = 0.0;
float ctz = 0.0;
float stx = 0.0;
float sty = 0.0;
float stz = 0.0;

float x_error = 0.0;
float y_error = 0.0;
float z_error = 0.0;
float xd_error = 0.0;
float yd_error = 0.0;
float zd_error = 0.0;

// straight line traj
float delta_x = 0.0;
float delta_y = 0.0;
float delta_z = 0.1;
float xa = 0.35;
float ya = 0.0;
float za = 0.3;
float xb = 0.35;
float yb = 0.0;
float zb = 0.4;

int t_total = 2;
//float speed_des = 0.2;

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

        // Desired step xyz generation
//        if ((mycount%4000) < 2000) {
//            xde = 0.40;
//            yde = 0.0;
//            zde = 0.35;
//
//        } else {
//            xde = 0.30;
//            yde = 0.15;
//            zde = 0.45;
//        }

    // Desired line wave generation
        if ((mycount%(2000*t_total)) < 1000.0*t_total) {
            xde = (-delta_x*(mycount%(2000*t_total)))/(t_total*1000.0) + xb;
            yde = (-delta_y*(mycount%(2000*t_total)))/(t_total*1000.0) + yb;
            zde = (-delta_z*(mycount%(2000*t_total)))/(t_total*1000.0) + zb;
        } else {
            xde = (delta_x*(mycount%(2000*t_total - 1000*t_total)))/(t_total*1000.0) + xa;
            yde = (delta_y*(mycount%(2000*t_total - 1000*t_total)))/(t_total*1000.0) + ya;
            zde = (delta_z*(mycount%(2000*t_total - 1000*t_total)))/(t_total*1000.0) + za;
        }


//    float* trajectory_array = trajectory((mycount%3000)/1000.0);
//    theta_desired = trajectory_array[0];
//    theta_desired_dot = trajectory_array[1];
//    theta_desired_ddot = trajectory_array[2];
//    free(trajectory_array);

    float* trajectory_fun = fun_trajectory((mycount)/1000.0);
    theta_desired1 = trajectory_fun[0];
    theta_desired2 = trajectory_fun[1];
    theta_desired3 = trajectory_fun[2];
    free(trajectory_fun);

//    if ((mycount)/1000 < 2) {
//        theta_desired1 = 0;
//        theta_desired2 = 0;
//        theta_desired3 = 0;
//    }

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

    // IIR
    xd = (x - x_old)/0.001;
    xd = (xd + xd_old1 + xd_old2)/3.0;
    x_old = x;
    xd_old2 = xd_old1;
    xd_old1 = xd;

    yd = (y - y_old)/0.001;
    yd = (yd + yd_old1 + yd_old2)/3.0;
    y_old = y;
    yd_old2 = yd_old1;
    yd_old1 = yd;

    zd = (z - z_old)/0.001;
    zd = (zd + zd_old1 + zd_old2)/3.0;
    z_old = z;
    zd_old2 = zd_old1;
    zd_old1 = zd;

    // Rotation zxy and its Transpose
    cosz = cos(thetaz);
    sinz = sin(thetaz);
    cosx = cos(thetax);
    sinx = sin(thetax);
    cosy = cos(thetay);
    siny = sin(thetay);
    RT11 = R11 = cosz*cosy-sinz*sinx*siny;
    RT21 = R12 = -sinz*cosx;
    RT31 = R13 = cosz*siny+sinz*sinx*cosy;
    RT12 = R21 = sinz*cosy+cosz*sinx*siny;
    RT22 = R22 = cosz*cosx;
    RT32 = R23 = sinz*siny-cosz*sinx*cosy;
    RT13 = R31 = -cosx*siny;
    RT23 = R32 = sinx;
    RT33 = R33 = cosx*cosy;

    // for impedance control
    ctx = cos(tx);
    cty = cos(ty);
    ctz = cos(tz);
    stx = sin(tx);
    sty = sin(ty);
    stz = sin(tz);
    x_error = xde-x;
    y_error = yde-y;
    z_error = zde-z;

    xd_error = xdd-xd;
    yd_error = ydd-yd;
    zd_error = zdd-zd;

    // Jacobian Transpose
    cosq1 = cos(theta1motor);
    sinq1 = sin(theta1motor);
    cosq2 = cos(theta2motor);
    sinq2 = sin(theta2motor);
    cosq3 = cos(theta3motor);
    sinq3 = sin(theta3motor);
    JT_11 = -0.254*sinq1*(cosq3 + sinq2);
    JT_12 = 0.254*cosq1*(cosq3 + sinq2);
    JT_13 = 0;
    JT_21 = 0.254*cosq1*(cosq2 - sinq3);
    JT_22 = 0.254*sinq1*(cosq2 - sinq3);
    JT_23 = -0.254*(cosq3 + sinq2);
    JT_31 = -0.254*cosq1*sinq3;
    JT_32 = -0.254*sinq1*sinq3;
    JT_33 = -0.254*cosq3;

    // Joint 1 calculating friction
    if (Omega1 > min_velocity1) {
        u_fric1 = Viscous_positive1*Omega1 + Coulomb_positive1;
    } else if (Omega1 < -min_velocity1) {
        u_fric1 = Viscous_negative1*Omega1 + Coulomb_negative1;
    } else {
        u_fric1 = slope*Omega1;
    }

    // Joint 2
    if (Omega2 > min_velocity2) {
        u_fric2 = Viscous_positive2*Omega2 + Coulomb_positive2;
    } else if (Omega2 < -min_velocity2) {
        u_fric2 = Viscous_negative2*Omega2 + Coulomb_negative2;
    } else {
        u_fric2 = slope*Omega2;
    }

    // Joint 3
    if (Omega3 > min_velocity3) {
        u_fric3 = Viscous_positive3*Omega3 + Coulomb_positive3;
    } else if (Omega3 < -min_velocity3) {
        u_fric3 = Viscous_negative3*Omega3 + Coulomb_negative3;
    } else {
        u_fric3 = slope*Omega3;
    }

    // multiplying by friction factors to not overestimate friction
    u_fric1 = u_fric1*fric_fac1;
    u_fric2 = u_fric2*fric_fac2;
    u_fric3 = u_fric3*fric_fac3;




    Zcmd = Zcmd_offset + Zcmd_force;
    if (fabs(theta_desired1 - theta1motor) > thresh) {
        // PD controller
        Ik1 = 0;
        Ik1_old = 0;
//       *tau1 = Kp1*(theta_desired-theta1motor)-Kd1*Omega1;
//        *tau1 = J1*theta_desired_ddot + Kp1*(theta_desired-theta1motor)+Kd1*(theta_desired_dot-Omega1);
//        *tau1 = Kp1*(theta_desired1-theta1motor)-Kd1*Omega1;
//        *tau1 = u_fric1;
//        *tau1 = -JT_11*(KPx*(x - xde) + KDx*(xd - xdd)) - JT_12*(KPy*(y - yde) + KDy*(yd - ydd)) - JT_13*(KPz*(z - zde) + KDz*(zd - zdd)) + u_fric1 + JT_13*Zcmd/Kt;
        *tau1 = (JT_11*(ctz*sty + cty*stx*stz) + JT_12*(sty*stz - cty*ctz*stx) + JT_13*ctx*cty)*(KDz*xd_error*(ctz*sty + cty*stx*stz) + KPz*x_error*(ctz*sty + cty*stx*stz) + KDz*yd_error*(sty*stz - cty*ctz*stx) + KPz*y_error*(sty*stz - cty*ctz*stx) + KDz*zd_error*ctx*cty + KPz*z_error*ctx*cty) + (JT_11*(cty*ctz - stx*sty*stz) + JT_12*(cty*stz + ctz*stx*sty) - JT_13*ctx*sty)*(KDx*xd_error*(cty*ctz - stx*sty*stz) + KPx*x_error*(cty*ctz - stx*sty*stz) + KDx*yd_error*(cty*stz + ctz*stx*sty) + KPx*y_error*(cty*stz + ctz*stx*sty) - KDx*zd_error*ctx*sty - KPx*z_error*ctx*sty) + (JT_13*stx + JT_12*ctx*ctz - JT_11*ctx*stz)*(KDy*zd_error*stx + KPy*z_error*stx + KDy*yd_error*ctx*ctz + KPy*y_error*ctx*ctz - KDy*xd_error*ctx*stz - KPy*x_error*ctx*stz) + u_fric1;

    } else {
        // PID controller
        error1 = theta_desired1 - theta1motor;

        Ik1 = Ik1_old + (error1 + error_old1)/2 * 0.001;
        error_old1 = error1;
//        *tau1 = Kpt1*(theta_desired-theta1motor)-Kdt1*Omega1 + Ki1*Ik1;
//        *tau1 = J1*theta_desired_ddot+ Kpt1*(theta_desired-theta1motor)+Kdt1*(theta_desired_dot-Omega1) + Ki1*Ik1;
//        *tau1 = Kpt1*(theta_desired1-theta1motor)-Kdt1*Omega1 + Ki1*Ik1;
//        *tau1 = u_fric1;
//        *tau1 = -JT_11*(KPx*(x - xde) + KDx*(xd - xdd)) - JT_12*(KPy*(y - yde) + KDy*(yd - ydd)) - JT_13*(KPz*(z - zde) + KDz*(zd - zdd)) + u_fric1 + JT_13*Zcmd/Kt;
        *tau1 = (JT_11*(ctz*sty + cty*stx*stz) + JT_12*(sty*stz - cty*ctz*stx) + JT_13*ctx*cty)*(KDz*xd_error*(ctz*sty + cty*stx*stz) + KPz*x_error*(ctz*sty + cty*stx*stz) + KDz*yd_error*(sty*stz - cty*ctz*stx) + KPz*y_error*(sty*stz - cty*ctz*stx) + KDz*zd_error*ctx*cty + KPz*z_error*ctx*cty) + (JT_11*(cty*ctz - stx*sty*stz) + JT_12*(cty*stz + ctz*stx*sty) - JT_13*ctx*sty)*(KDx*xd_error*(cty*ctz - stx*sty*stz) + KPx*x_error*(cty*ctz - stx*sty*stz) + KDx*yd_error*(cty*stz + ctz*stx*sty) + KPx*y_error*(cty*stz + ctz*stx*sty) - KDx*zd_error*ctx*sty - KPx*z_error*ctx*sty) + (JT_13*stx + JT_12*ctx*ctz - JT_11*ctx*stz)*(KDy*zd_error*stx + KPy*z_error*stx + KDy*yd_error*ctx*ctz + KPy*y_error*ctx*ctz - KDy*xd_error*ctx*stz - KPy*x_error*ctx*stz) + u_fric1;


    }

    if (fabs(theta_desired2 - theta2motor) > thresh) {
        // PD controller
        Ik2 = 0;
        Ik2_old = 0;
//        *tau2 = Kp2*(theta_desired-theta2motor)-Kd2*Omega2;
//        *tau2 = J2*theta_desired_ddot + Kp2*(theta_desired-theta2motor)+Kd2*(theta_desired_dot-Omega2);
//        *tau2 = Kp2*(theta_desired2-theta2motor)-Kd2*Omega2;
//        *tau2 = u_fric2;
//        *tau2 =  -JT_21*(KPx*(x - xde) + KDx*(xd - xdd)) - JT_22*(KPy*(y - yde) + KDy*(yd - ydd)) - JT_23*(KPz*(z - zde) + KDz*(zd - zdd)) + u_fric2 + JT_23*Zcmd/Kt;
        *tau2 = (JT_21*(ctz*sty + cty*stx*stz) + JT_22*(sty*stz - cty*ctz*stx) + JT_23*ctx*cty)*(KDz*xd_error*(ctz*sty + cty*stx*stz) + KPz*x_error*(ctz*sty + cty*stx*stz) + KDz*yd_error*(sty*stz - cty*ctz*stx) + KPz*y_error*(sty*stz - cty*ctz*stx) + KDz*zd_error*ctx*cty + KPz*z_error*ctx*cty) + (JT_21*(cty*ctz - stx*sty*stz) + JT_22*(cty*stz + ctz*stx*sty) - JT_23*ctx*sty)*(KDx*xd_error*(cty*ctz - stx*sty*stz) + KPx*x_error*(cty*ctz - stx*sty*stz) + KDx*yd_error*(cty*stz + ctz*stx*sty) + KPx*y_error*(cty*stz + ctz*stx*sty) - KDx*zd_error*ctx*sty - KPx*z_error*ctx*sty) + (JT_23*stx + JT_22*ctx*ctz - JT_21*ctx*stz)*(KDy*zd_error*stx + KPy*z_error*stx + KDy*yd_error*ctx*ctz + KPy*y_error*ctx*ctz - KDy*xd_error*ctx*stz - KPy*x_error*ctx*stz) + u_fric2;

    } else {
        // PID controller
        error2 = theta_desired2 - theta2motor;
        Ik2 = Ik2_old + (error2 + error_old2)/2 * 0.001;
        error_old2 = error2;
//        *tau2 = Kpt2*(theta_desired-theta2motor)-Kdt2*Omega2 + Ki2*Ik2;
//        *tau2 = J2*theta_desired_ddot+ Kpt2*(theta_desired-theta2motor)+Kdt2*(theta_desired_dot-Omega2) + Ki2*Ik2;
//        *tau2 = Kpt2*(theta_desired2-theta2motor)-Kdt2*Omega2 + Ki2*Ik2;
//        *tau2 = u_fric2;
//        *tau2 =  -JT_21*(KPx*(x - xde) + KDx*(xd - xdd)) - JT_22*(KPy*(y - yde) + KDy*(yd - ydd)) - JT_23*(KPz*(z - zde) + KDz*(zd - zdd)) + u_fric2 + JT_23*Zcmd/Kt;
        *tau2 = (JT_21*(ctz*sty + cty*stx*stz) + JT_22*(sty*stz - cty*ctz*stx) + JT_23*ctx*cty)*(KDz*xd_error*(ctz*sty + cty*stx*stz) + KPz*x_error*(ctz*sty + cty*stx*stz) + KDz*yd_error*(sty*stz - cty*ctz*stx) + KPz*y_error*(sty*stz - cty*ctz*stx) + KDz*zd_error*ctx*cty + KPz*z_error*ctx*cty) + (JT_21*(cty*ctz - stx*sty*stz) + JT_22*(cty*stz + ctz*stx*sty) - JT_23*ctx*sty)*(KDx*xd_error*(cty*ctz - stx*sty*stz) + KPx*x_error*(cty*ctz - stx*sty*stz) + KDx*yd_error*(cty*stz + ctz*stx*sty) + KPx*y_error*(cty*stz + ctz*stx*sty) - KDx*zd_error*ctx*sty - KPx*z_error*ctx*sty) + (JT_23*stx + JT_22*ctx*ctz - JT_21*ctx*stz)*(KDy*zd_error*stx + KPy*z_error*stx + KDy*yd_error*ctx*ctz + KPy*y_error*ctx*ctz - KDy*xd_error*ctx*stz - KPy*x_error*ctx*stz) + u_fric2;

    }

    if (fabs(theta_desired3 - theta3motor) > thresh) {
        // PD controller
        Ik3 = 0;
        Ik3_old = 0;
//        *tau3 = Kp3*(theta_desired-theta3motor)-Kd3*Omega3;
//        *tau3 = J3*theta_desired_ddot + Kp3*(theta_desired-theta3motor)+Kd3*(theta_desired_dot-Omega3);
//        *tau3 = Kp3*(theta_desired3-theta3motor)-Kd3*Omega3;
//        *tau3 = u_fric3;
//        *tau3 = -JT_31*(KPx*(x - xde) + KDx*(xd - xdd)) - JT_32*(KPy*(y - yde) + KDy*(yd - ydd)) - JT_33*(KPz*(z - zde) + KDz*(zd - zdd)) + u_fric3 + JT_33*Zcmd/Kt;
        *tau3 = (JT_31*(ctz*sty + cty*stx*stz) + JT_32*(sty*stz - cty*ctz*stx) + JT_33*ctx*cty)*(KDz*xd_error*(ctz*sty + cty*stx*stz) + KPz*x_error*(ctz*sty + cty*stx*stz) + KDz*yd_error*(sty*stz - cty*ctz*stx) + KPz*y_error*(sty*stz - cty*ctz*stx) + KDz*zd_error*ctx*cty + KPz*z_error*ctx*cty) + (JT_31*(cty*ctz - stx*sty*stz) + JT_32*(cty*stz + ctz*stx*sty) - JT_33*ctx*sty)*(KDx*xd_error*(cty*ctz - stx*sty*stz) + KPx*x_error*(cty*ctz - stx*sty*stz) + KDx*yd_error*(cty*stz + ctz*stx*sty) + KPx*y_error*(cty*stz + ctz*stx*sty) - KDx*zd_error*ctx*sty - KPx*z_error*ctx*sty) + (JT_33*stx + JT_32*ctx*ctz - JT_31*ctx*stz)*(KDy*zd_error*stx + KPy*z_error*stx + KDy*yd_error*ctx*ctz + KPy*y_error*ctx*ctz - KDy*xd_error*ctx*stz - KPy*x_error*ctx*stz) + u_fric3;
    } else {
        // PID controller
        error3 = theta_desired3 - theta3motor;
//        Ik3 = Ik3 + (error3 + error_old3)/2 * 0.001;
        Ik3 = Ik3_old + (error3 + error_old3)/2 * 0.001;
        error_old3 = error3;
//        *tau3 = Kpt3*(theta_desired-theta3motor)-Kdt3*Omega3 + Ki3*Ik3;
//        *tau3 = J3*theta_desired_ddot+ Kpt3*(theta_desired-theta3motor)+Kdt3*(theta_desired_dot-Omega3) + Ki3*Ik3;
//        *tau3 = Kpt3*(theta_desired3-theta3motor)-Kdt3*Omega3 + Ki3*Ik3;
//        *tau3 = u_fric3;
//        *tau3 = -JT_31*(KPx*(x - xde) + KDx*(xd - xdd)) - JT_32*(KPy*(y - yde) + KDy*(yd - ydd)) - JT_33*(KPz*(z - zde) + KDz*(zd - zdd)) + u_fric3 + JT_33*Zcmd/Kt;
        *tau3 = (JT_31*(ctz*sty + cty*stx*stz) + JT_32*(sty*stz - cty*ctz*stx) + JT_33*ctx*cty)*(KDz*xd_error*(ctz*sty + cty*stx*stz) + KPz*x_error*(ctz*sty + cty*stx*stz) + KDz*yd_error*(sty*stz - cty*ctz*stx) + KPz*y_error*(sty*stz - cty*ctz*stx) + KDz*zd_error*ctx*cty + KPz*z_error*ctx*cty) + (JT_31*(cty*ctz - stx*sty*stz) + JT_32*(cty*stz + ctz*stx*sty) - JT_33*ctx*sty)*(KDx*xd_error*(cty*ctz - stx*sty*stz) + KPx*x_error*(cty*ctz - stx*sty*stz) + KDx*yd_error*(cty*stz + ctz*stx*sty) + KPx*y_error*(cty*stz + ctz*stx*sty) - KDx*zd_error*ctx*sty - KPx*z_error*ctx*sty) + (JT_33*stx + JT_32*ctx*ctz - JT_31*ctx*stz)*(KDy*zd_error*stx + KPy*z_error*stx + KDy*yd_error*ctx*ctz + KPy*y_error*ctx*ctz - KDy*xd_error*ctx*stz - KPy*x_error*ctx*stz) + u_fric3;

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

    Simulink_PlotVar1 = x;
    Simulink_PlotVar2 = xde;
    Simulink_PlotVar3 = y;
    Simulink_PlotVar4 = yde;

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


    Ik1_old = Ik1;
    Ik2_old = Ik2;
    Ik3_old = Ik3;
    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
//        serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);


//        serial_printf(&SerialA, "Motor Thetas: %.2f %.2f %.2f \n\r", printtheta1motor*180/PI, printtheta2motor*180/PI, printtheta3motor*180/PI);
//        serial_printf(&SerialA, "Position: %.2f %.2f %.2f  \n\r", x, y, z);
//        serial_printf(&SerialA, "IK DH Thetas: %.2f %.2f %.2f   \n\r", IKTh1*180/PI, IKTh2*180/PI, IKTh3*180/PI);
//        serial_printf(&SerialA, "Motor Theta (IK)s: %.2f %.2f %.2f  \n\r", IKtheta1motor*180/PI, IKtheta2motor*180/PI, IKtheta3motor*180/PI );
//        serial_printf(&SerialA, "----------------\n\r");

        serial_printf(&SerialA, "Position: %.2f %.2f %.2f  \n\r", theta_desired1, theta_desired2, theta_desired3);
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
//    if (t>= 0 && t<=) {
//        float x = 0.25;
//        float y = 0;
//        float z1 = (0.50-0.35)*((t-t0)/t_total) + 0.35;
//        theta_desired1 = atan2(y,x);
//        theta_desired2 = atan2((0.254-z),sqrt(pow(x,2) + pow(y,2)))-acos( (pow(y,2)+pow(x,2)+pow((0.254-z),2)) / (2*(sqrt(pow(x,2)+pow(y,2)+pow((0.254-z),2))*(0.254))));
//        theta_desired3 = PI - acos( (-(pow((0.254-z),2)+pow(x,2)+pow(y,2)) + pow((0.254),2) + pow((0.254),2)) / (2*.254*.254) );
//        }
//    else if (t > 1 && t<=2 ) {
//        float x = 0.25;
//        float y = 0;
//        float z2 = (0.35-0.5)*((t-t02)/t_total) + 0.5;
//        theta_desired1 = atan2(y,x);
//        theta_desired2 = atan2((0.254-z),sqrt(pow(x,2) + pow(y,2)))-acos( (pow(y,2)+pow(x,2)+pow((0.254-z),2)) / (2*(sqrt(pow(x,2)+pow(y,2)+pow((0.254-z),2))*(0.254))));
//        theta_desired3 = PI - acos( (-(pow((0.254-z),2)+pow(x,2)+pow(y,2)) + pow((0.254),2) + pow((0.254),2)) / (2*.254*.254) );
//    }

    float y_max = 0.15;
    float y_min = y_max*(-1);
    float z_max = 0.5334;
    float z_min = 0.127;
    float z = (z_max - z_min)*cos(PI*t/3.5)/2 + (z_max + z_min)/2;
    float y = (y_max - y_min)*sin(2*PI*t/3.5)/2 + (y_max + y_min)/2;
    float x = 0.3937;

    theta_desired1 = atan2(y,x);
    theta_desired2 = atan2((0.254-z),sqrt(pow(x,2) + pow(y,2)))-acos( (pow(y,2)+pow(x,2)+pow((0.254-z),2)) / (2*(sqrt(pow(x,2)+pow(y,2)+pow((0.254-z),2))*(0.254))));
    theta_desired3 = PI - acos( (-(pow((0.254-z),2)+pow(x,2)+pow(y,2)) + pow((0.254),2) + pow((0.254),2)) / (2*.254*.254) );
    
    float IKtheta1motor1 = theta_desired1;
    float IKtheta2motor2 = theta_desired2 + PI/2;
    float IKtheta3motor3 = theta_desired3 + IKtheta2motor2 - PI/2;

    *res = IKtheta1motor1;
    *(res + 1) = IKtheta2motor2;
    *(res + 2) = IKtheta3motor3;
    return res;
}
