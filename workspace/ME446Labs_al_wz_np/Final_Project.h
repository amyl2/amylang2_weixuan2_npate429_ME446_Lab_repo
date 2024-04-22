#include "math.h"
#include <stdbool.h>
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81


typedef struct {
    float xDes;
    float yDes;
    float zDes;
    float time;
    float theta_z;
    float force;
} Waypoint;


/** ********** Encoder Offsets ********** **/
// These two offsets are only used in the main file user_CRSRobot.c
// You just need to create them here and find the correct offset and then these offset will adjust the encoder readings.
float offset_Enc2_rad = -0.42;
float offset_Enc3_rad = 0.23;

/** ********** Global variables ********** **/
/** Initialization Variables */
// The variable mycount keeps tract of the number of times the lab function has been run.
long mycount = 0;

// Initialization of theta array and necessary variables to store past 100 motor theta values.
#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;
#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];
#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];
long arrayindex = 0;
int UARTprint = 0;

/** Variables to store theta motor values to print */
float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

/** Initialization of float values to plot in Simulink */
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

/** Denavit-Hartenberg Thetas */
float Th2DH = 0;
float Th3DH = 0;

/** Inverse-Kinematics Thetas */
// Denavit-Hartenberg IK Thetas
float IKTh1 = 0;
float IKTh2 = 0;
float IKTh3 = 0;

// Motor IK Thetas
float IKtheta1motor = 0;
float IKtheta2motor = 0;
float IKtheta3motor = 0;

/** Position Variables */
float x = 0;
float y = 0;
float z = 0;

/** Infinite Impulse Response (IIR) variables */
// for motor 1
float Theta1_old = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;

// for motor 2
float Theta2_old = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;

// for motor 3
float Theta3_old = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;

/** Friction Constants and Parameters */
// Viscous and Coulomb Terms (Positive)
float Viscous_positive1 = 0.195;
float Viscous_positive2 = 0.2500;
float Viscous_positive3 = 0.1922;
float Coulomb_positive1 = 0.3405;
float Coulomb_positive2 = 0.1675;
float Coulomb_positive3 = 0.17039;

// Viscous and Coulomb Terms (Negative)
float Viscous_negative1 = 0.195;
float Viscous_negative2 = 0.2870;
float Viscous_negative3 = 0.2132;
float Coulomb_negative1 = -0.3405;
float Coulomb_negative2 = -0.16565;
float Coulomb_negative3 = -0.16934;

// Slope value for Omega vs u proportional control effort
float slope = 3.6;

// min velocity values for the joints
float min_velocity1 = 0.1;
float min_velocity2 = 0.05;
float min_velocity3 = 0.05;

// Friction Compensation variables
float u_fric1 = 0.0;
float u_fric2 = 0.0;
float u_fric3 = 0.0;

// Friction Multiplication Factors used in Feed Forward Control
float fric_fac1 = 0.45;
float fric_fac2 = 1.0;
float fric_fac3 = 1.7;

/** Task Space PD Control */
// KP gains for x, y, and z
float KPx = 280.0; // 280.0
float KPy = 150.0; // 150.0
float KPz = 220.0; // 220.0

// KD gains for x, y, and z
float KDx = 19.0; // 19.0
float KDy = 12.0; //12.0
float KDz = 12.0; //12.0

// x, y and z desired values in meters
float xde = 0.0;
float yde = 0.0;
float zde = 0.0;

// x, y and z DOT desired values
float xdd = 0.0;
float ydd = 0.0;
float zdd = 0.0;

// x, y and z DOT current values
float xd = 0.0;
float yd = 0.0;
float zd = 0.0;

// used to calculate x, y, and z DOT current values through IIR
float x_old = 0.15;
float xd_old1 = 0.0;
float xd_old2 = 0.0;
float y_old = 0.01;
float yd_old1 = 0.0;
float yd_old2 = 0.0;
float z_old = 0.43;
float zd_old1 = 0.0;
float zd_old2 = 0.0;

// These JT variables together represent the Jacobian Transpose Matrix [3x3]
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;

// Help compute Jacobian Transpose
float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;

// Rotation variables
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;

// Transpose Rotation variables
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;

// Help compute Rotation Matrix
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;

// Rotation thetas about x, y, and z
float thetaz = 0;
float thetax = 0;
float thetay = 0;

/** Feed Forward Force Terms */
// Variables to offset gravity and external forces.
float Zcmd_offset = 10;
float Zcmd_force = 0;
float Zcmd = 0;

// Robot Torque Constant Kt
float Kt = 6.0;

/** Impedance Control (Axis Rotation) */
// Theta values to rotate axis by
float tx = 0.0;
float ty = 0.0;
float tz = 0.0;

// cos and sin varialbes for calculations
float ctx = 0.0;
float cty = 0.0;
float ctz = 0.0;
float stx = 0.0;
float sty = 0.0;
float stz = 0.0;

// x, y, and z error variables
float x_error = 0.0;
float y_error = 0.0;
float z_error = 0.0;

// x_dot, y_dot, and z_dot error variables
float xd_error = 0.0;
float yd_error = 0.0;
float zd_error = 0.0;

/** Straight Line Trajectory Variables */
// Desired change in x, y, z positions [meters]
float delta_x = 0.0;
float delta_y = 0.0;
float delta_z = 0.1;

// Desired  locations for a and b positions [meters]
float xa = 0.35;
float ya = 0.0;
float za = 0.3;
float xb = 0.35;
float yb = 0.0;
float zb = 0.4;

// Total time desired for each trajectory cycle
int t_total = 2;

//* Insert Peg Step */
Waypoint wp1 = {0.15, 0.01, 0.43, 0.0, 0.0, 0.0};// INIT
Waypoint wp2 = {0.17, 0.19, 0.56, 1.0, 0.0, 0.0};// Straight Line Out
Waypoint wp3 = {0.0325, 0.345, 0.56, 2.0, 0.0, 0.0};// Center over peg
Waypoint wp4 = {0.0325, 0.345, 0.20, 3.0, 0.0, 0.0};// "" but slightly lower
Waypoint wp5 = {0.0325, 0.345, 0.115, 4.0, 0.0, 0.0};// "" but EVEN lower
Waypoint wp6 = {0.0325, 0.345, 0.115, 5.0, 0.0, 0.0};// Hold
Waypoint wp7 = {0.0325, 0.345, 0.35, 6.0, 0.0, 0.0};// Rise above

//* Maze Step */
Waypoint wp8 = {0.255, 0.110, 0.35, 7.0, 0.0, 0.0}; // above maze
Waypoint wp9 = {0.402, 0.110, 0.205, 8.0, 0.0, 0.0}; // at maze
Waypoint wp10 = {0.426, 0.064, 0.205, 9.0, -(90.0-36.87)*PI/180.0, 0.0}; // first line
Waypoint wp11 = {0.427, 0.053, 0.205, 9.1, 0, 0.0}; // transition first line
Waypoint wp12 = {0.417, 0.042, 0.205, 9.2, -(45)*PI/180.0, 0.0}; // transition #2 first line
Waypoint wp13 = {0.403, 0.042, 0.205, 9.3, (15)*PI/180.0, 0.0}; // transition #3
Waypoint wp14 = {0.354, 0.054, 0.205, 9.4, -(15)*PI/180.0, 0.0}; // line #2
Waypoint wp15 = {0.347, 0.052, 0.205, 9.5, -(15)*PI/180.0, 0.0}; // transition line 2 first
Waypoint wp16 = {0.315, 0.045, 0.205, 9.6, -(45)*PI/180.0, 0.0}; // transition line 2 second // 0.330 x originally
Waypoint wp17 = {0.330, 0.031, 0.205, 9.7, -(0)*PI/180.0, 0.0}; // transition line 2 third
Waypoint wp18 = {0.398, -0.054, 0.205, 10.0, -(90.0-36.87)*PI/180.0, 0.0}; // line 3





/** ********** Function Declaration ********** **/
void initialization(float t1m,float t2m);

void mains_code(void);

void Infinite_Impulse_Response(float theta1motor,float theta2motor,float theta3motor);
void Rot_FF_IC_calc(float theta1motor,float theta2motor,float theta3motor);
void Friction();
void Motor_Control(float *tau1,float *tau2,float *tau3);
void Torque_Saturation(float *tau1,float *tau2,float *tau3);
void Updating_Vars(float theta1motor,float theta2motor,float theta3motor);
void Forwards_Kinematics(float theta1motor,float theta2motor,float theta3motor);
void Inverse_Kinematics();
void Output_Vars(float theta1motor,float theta2motor,float theta3motor);
void Set_Imp_Gains(float KPx_new, float KPy_new, float KPz_new, float KDx_new, float KDy_new, float KDz_new, bool reset);

void StraightLine(Waypoint init, Waypoint final);
void Hold(Waypoint hold_point);
