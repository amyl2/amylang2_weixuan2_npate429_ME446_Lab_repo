#include "math.h"
#include <stdbool.h>
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81


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

/** PD Controller Constants */
// Kp constants for motor 1 (M1), motor 2 (M2), and motor 3 (M3) respectively
float Kp1 = 20.0;
float Kp2 = 75;
float Kp3 = 65;

// Kd constants for motor 1, motor 2, and motor 3 respectively
float Kd1 = 1.35;
float Kd2 = 2.35;
float Kd3 = 2.0;

/** PID Control Constants */
// threshhold value for which to begin intergal control
float thresh = 0.03;

// Kp and Kd constants for M1, M2, & M3 using integral control
//
// note: These values are different than previous PD controller constant values.
//       This is due to the threshold criteria of the PID contorller. Because PID
//       control is only used under a small threshold value, Kp and Kd terms must
//       be increased in order for them to have the desired impact on the motor.
float Kpt1 = 150;
float Kpt2 = 300;
float Kpt3 = 250;
float Kdt1 = 2.1;
float Kdt2 = 8;
float Kdt3 = 4;

// Ki constants for the integral term in PID control
float Ki1 = 175;
float Ki2 = 1000;
float Ki3 = 2000;

/** PID Controller Variables */
// curr and new integral terms for PID controller
float Ik1 = 0;
float Ik2 = 0;
float Ik3 = 0;
float Ik1_old = 0;
float Ik2_old = 0;
float Ik3_old = 0;

// curr and old error terms for PID controller
float error1 = 0;
float error2 = 0;
float error3 = 0;
float error_old1 = 0;
float error_old2 = 0;
float error_old3 = 0;

/** Infinite Impulse Response (IIR) variables */
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

/** Feed Forward variables */
float t = 0.0;
float J1 = 0.0167;
float J2 = 0.03;
float J3 = 0.0128;
float theta_desired_dot = 0.0;
float theta_desired_ddot = 0.0;

/** Fun Trajectory Theta Desired Variables */
float theta_desired1 = 0.0;
float theta_desired2 = 0.0;
float theta_desired3 = 0.0;

/** Section True/False Parameters */
bool cubic_wave = false;
bool fun = false;

/** ********** Function Declaration ********** **/
void initialization(float t1m,float t2m);
void fun_trajectory(float t);
void square_wave_trajectory(float t);
void cubic_wave_trajectory(float t);

void mains_code(void);

/** ********** Main ********** **/
void main(void)
{
	mains_code();
}


// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3) {
    // necessary initialization
    initialization(theta1motor, theta2motor);

    /** Creating desired waves 
     * 
     * This section deals with generating trajectories or desired waveforms that the CRS
     * Robot Arm will follow during its operation. 
     * 
     * Toggle commenting statements in order to choose the trajectory to follow! 
     * By uncommenting or commenting specific lines of code, you can select different
     * trajectory profiles such as square wave, cubic wave, or other fun waveforms.
     *  
     * Note: Only one trajectory should be uncommented at all times to ensure that the
     * robot arm follows the intended path. Mixing multiple trajectories or waveforms
     * can lead to unpredictable or undesired motion.
    */

    /* uncomment below lines to have the CRS Robot Arm follow a square-wave trajectory! */
    // t = (mycount%3000)/1000;
    // square_wave_trajectory(t);

    /* uncomment below lines to have the CRS Robot Arm follow a cubic-wave trajectory! */
    // cubic_wave = true;
    // t = (mycount%3000)/1000;
    // cubic_wave_trajectory(t);

    /* uncomment below lines to have the CRS Robot Arm follow a fun-wave trajectory! */
    fun = true;
    t = (mycount)/1000;
    fun_trajectory(t);


    /** Infinite Impulse Response
     * 
     * This section implements the Infinite Impulse Response (IIR) filtering technique
     * to calculate raw angular velocity (Omega) optimally.
     * 
     * Omega, representing angular velocity, is first found by using a simple difference
     * in thetas equation for a basic estimation. Afterwards, an Infinite Impulse Response (IIR) 
     * averaging filter is used to filer and smoothen the omega calculation.
     * 
     * This ensures that the Omega values used for later control calculations are more
     * stable with smoother fluxuations which allows for more stable and precise control 
     * of the CRS Robot Arm.
     */

    // First simple Omega calculations using theta differences.
    Omega1 = (theta1motor - Theta1_old)/0.001;
    Omega2 = (theta2motor - Theta2_old)/0.001;
    Omega3 = (theta3motor - Theta3_old)/0.001;

    // implementing averaging filter for Omegas
    Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0;
    Omega2 = (Omega2 + Omega2_old1 + Omega2_old2)/3.0;
    Omega3 = (Omega3 + Omega3_old1 + Omega3_old2)/3.0;

    /** PD & PID control
     * 
     * Here we implement both PD and PID control for regulating the CRS Robot Arm's movements.
     *  If the difference between our desired theta and actual motor theta is greater than a 
     * certain threshold, we use PD control. Otherwise we use PID control. Essentially, PD control
     * is used the majority of the time but as the CRS robot arm gets closer to the desired theta
     * and error becomes smaller, we use PID control.
     * 
     * PD Control: 
     * PD control calculates the contorl torque based on proportional and derivative terms, and
     * it's used to reduce the error quickly while damping oscillations. PD control begins by first
     * resetting the integral term constants from PID control to 0. Afterwards, the torque values
     * are calculated and set.
     * 
     * PID Control: 
     * PID control further builds upon PD control by adding an integral term to the calculation
     * which integrates error over time. This helps reduce steady-state error and improves the 
     * overall accuracy of our mapping.
     * 
     * Additionally the torque values are set differently when using feed forward control. Because
     * we are only using feed forwards if using our cubic trajectory, we have added a cubic_wave flag
     * to do so. In this case, feedforward constants J1, J2, and J3 are used and additional terms are
     * added to enhance control performance. 
     * 
     * Local Variables : 
     *              curr_desired1  : Current Desired Theta Value for Motor 1 (rad) 
     *              curr_desired2  : Current Desired Theta Value for Motor 2 (rad) 
     *              curr_desired3  : Current Desired Theta Value for Motor 3 (rad) 
     * 
     *      Note: For different cases of desired wave functions, different variables 
     *            and logic are used. In the case of the fun trajectory, we want our
     *            3 motors to be set to distinct unique waveforms for the desired path.
     *            For the cubic and square waves, however, we just have one theta desired
     *            for all our motors. The curr_desired variables store this data internally.
    */

    /* if choosing fun trajectory then different theta_desired values are set for EACH motor */
    /* else only one theta_desired */
    float curr_desired1 = fun ? theta_desired1 : theta_desired;
    float curr_desired2 = fun ? theta_desired2 : theta_desired;
    float curr_desired3 = fun ? theta_desired3 : theta_desired;

    /* Motor 1 */
    if (fabs(curr_desired1 - theta1motor) > thresh) { // PD controller
        Ik1 = 0;      // first reset the integral term values to 0
        Ik1_old = 0;
        if (cubic_wave) {  // if using a cubic wave then use feed forward control
            *tau1 = J1*theta_desired_ddot + Kp1*(theta_desired-theta1motor) + Kd1*(theta_desired_dot-Omega1);
        } else {
            *tau1 = Kp1*(curr_desired1-theta1motor) - Kd1*Omega1;
        }
    } else { // PID controller
        error1 = curr_desired1 - theta1motor;
        Ik1 = Ik1_old + (error1 + error_old1)/2 * 0.001;
        error_old1 = error1;
        if (cubic_wave) {  // if using a cubic wave then use feed forward control
            *tau1 = J1*theta_desired_ddot+ Kpt1*(theta_desired-theta1motor)+Kdt1*(theta_desired_dot-Omega1) + Ki1*Ik1;
        } else {
            *tau1 = Kpt1*(curr_desired1-theta1motor)-Kdt1*Omega1 + Ki1*Ik1;
        }
    }

    /* Motor 2 */
    if (fabs(curr_desired2 - theta2motor) > thresh) { // PD controller
        Ik2 = 0;
        Ik2_old = 0;
        if (cubic_wave) {  // if using a cubic wave then use feed forward control
            *tau2 = J2*theta_desired_ddot + Kp2*(theta_desired-theta2motor) + Kd2*(theta_desired_dot-Omega2);
        } else {
            *tau2 = Kp2*(curr_desired2-theta2motor) - Kd2*Omega2;
        }
    } else { // PID controller
        error2 = curr_desired2 - theta2motor;
        Ik2 = Ik2_old + (error2 + error_old2)/2 * 0.001;
        error_old2 = error2;
        if (cubic_wave) {  // if using a cubic wave then use feed forward control
            *tau2 = J2*theta_desired_ddot+ Kpt2*(theta_desired-theta2motor)+Kdt2*(theta_desired_dot-Omega2) + Ki2*Ik2;
        } else {
            *tau2 = Kpt2*(curr_desired2-theta2motor)-Kdt2*Omega2 + Ki2*Ik2;
        }
    }

    /* Motor 3 */
    if (fabs(curr_desired3 - theta3motor) > thresh) { // PD controller
        Ik3 = 0;
        Ik3_old = 0;
        if (cubic_wave) {  // if using a cubic wave then use feed forward control
            *tau3 = J3*theta_desired_ddot + Kp3*(theta_desired-theta3motor) + Kd3*(theta_desired_dot-Omega3);
        } else {
            *tau3 = Kp3*(curr_desired3-theta3motor) - Kd3*Omega3;
        }
    } else { // PID controller
        error3 = theta_desired3 - theta3motor;
        Ik3 = Ik3_old + (error3 + error_old3)/2 * 0.001;
        error_old3 = error3;
        if (cubic_wave) {  // if using a cubic wave then use feed forward control
            *tau3 = J3*theta_desired_ddot+ Kpt3*(theta_desired-theta3motor)+Kdt3*(theta_desired_dot-Omega3) + Ki3*Ik3;
        } else {
            *tau3 = Kpt3*(curr_desired3-theta3motor)-Kdt3*Omega3 + Ki3*Ik3;
        }
    }


    /** Saturate the Torque Values
     * 
     * Because all real motors have limited torque capability,
     * it is important to saturate the torque values to prevent
     * excessive stress on the motors and maintain safe operation
     * of the robot.
     * 
     * This snippit caps the torque value at +/- 5.
     * 
     * Furthermore, because we cap the torque, it is important to 
     * stop integrating if using PID control. Resultingly, if
     * PID control is being used, we reset the Ik integral terms
     * to their previous values.
     * 
     * Note: even if PID controller is not in use, resetting Ik1 values
     *       will have no adverse effect on the system.
     */

    /* Motor 1 */
    if (*tau1 > 5.0){
        *tau1 = 5.0;
        Ik1 = Ik1_old;
    } else if ( *tau1 < -5.0 ){
        *tau1 = -5.0;
        Ik1 = Ik1_old;
    }

    /* Motor 2 */
    if (*tau2 > 5.0){
        *tau2 = 5.0;
        Ik2 = Ik2_old;
    } else if ( *tau2 < -5.0 ){
        *tau2 = -5.0;
        Ik2 = Ik2_old;
    }

    /* Motor 3 */
    if (*tau3 > 5.0){
        *tau3 = 5.0;
        Ik3 = Ik3_old;
    } else if ( *tau3 < -5.0 ){
        *tau3 = -5.0;
        Ik3 = Ik3_old;
    }

    /** Output Variables
     * 
     * Often it is important to see what your data/values look like
     * in order to debug or verify that your CRS robot is moving as
     * intended. Here we set those variables for TeraTerm and Simulink
     * respectively.
     * 
     * These variables are subject to change depending on what information
     * you want to view.
     * 
     * variables:
     *        - printtheta1motor, printtheta2motor, printtheta3motor
     *              - These are variables sent to print to TeraTerm.
     *              - Can be changed depending on needs.
     *        - Simulink_PlotVar1, Simulink_PlotVar2, Simulink_PlotVar3, Simulink_PlotVar4
     *              - Variables to send to Simulink for analysis.
     */
    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;

    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
    Simulink_PlotVar4 = theta_desired;

    /* Uncomment this below snippit to plot the difference between 
       desired thetas and actual motor thetas instead */
    
    // Simulink_PlotVar1 = theta_desired-theta1motor;
    // Simulink_PlotVar2 = theta_desired-theta2motor;
    // Simulink_PlotVar3 = theta_desired-theta3motor;
    // Simulink_PlotVar4 = theta_desired-theta_desired;

    /** Forward Kinematics (simulation)
     * 
     * Although this code is not involved in controlling the CRS
     * robot arm, it is imporant to solve out the forwards kinematics
     * in order to calculate check that the positions and DH angles 
     * of the end-effector are as expected.
     * 
     * We define x,y,z positions [meters] as well as the DH theta values [rad]
     * based on our previous calculations in lab 1.
    */
    x = 0.254*cos(theta1motor)*(cos(theta3motor)+sin(theta2motor));
    y = 0.254*sin(theta1motor)*(cos(theta3motor)+sin(theta2motor));
    z = 0.254*cos(theta2motor)-0.254*sin(theta3motor)+0.254;

    Th2DH = theta2motor - PI/2;
    Th3DH = -theta2motor + theta3motor + PI/2;

    /** Inverse Kinematics (control)
     * 
     * Although this code is not used in controlling the CRS
     * robot arm, it is imporant to solve out the inverse kinematics
     * in order to calculate check that the motor angles are as expected.
     * 
     * We define DH theta values [rad] as well as motor theta values [rad]
     * for all three motors based on our previous calculations done in lab 1.
    */
    IKTh1 = atan2(y,x);
    IKTh2 = atan2((0.254-z),sqrt(pow(x,2) + pow(y,2)))-acos( (pow(y,2)+pow(x,2)+pow((0.254-z),2)) / (2*(sqrt(pow(x,2)+pow(y,2)+pow((0.254-z),2))*(0.254))));
    IKTh3 = PI - acos( (-(pow((0.254-z),2)+pow(x,2)+pow(y,2)) + pow((0.254),2) + pow((0.254),2)) / (2*.254*.254) );

    IKtheta1motor = IKTh1;
    IKtheta2motor = IKTh2 + PI/2;
    IKtheta3motor = IKTh3 + IKtheta2motor - PI/2;

    /** Updating variables for next iteration of the loop 
     * 
     * At the end of each iteration of the loop, it is imperative to update
     * variables storing old values for the sake of continuity. 
     * 
     * Specifically, we update old theta values, old omega values, as well as
     * old integral terms for PID control. 
     * 
     * At the end, we increase the value of mycount to keep track of the number
     * of iterations of the loop that are completed. This is used in the loop for
     * time based calculations so keeping this value up to date is essential.
    */

    // for Omega1
    Theta1_old = theta1motor;
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;
    // for Omega2
    Theta2_old = theta2motor;
    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;
    // for Omega3
    Theta3_old = theta3motor;
    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;

    // for PID control
    Ik1_old = Ik1;
    Ik2_old = Ik2;
    Ik3_old = Ik3;

    // for overall loop
    mycount++;
}

void printing(void){
    /** Printing Loop for TeraTerm 
     * 
     * This function sends serial_printf statements to print to TeraTerm. Different
     * lines can be un/commented in order to print different variables. 
     * 
     * This step is essential for the debugging process as it allows the user to cross
     * check the expected values with the actual values in real time to see where the code
     * might be going wrong. 
    */

    if (whattoprint == 0) {
        // serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);
        // serial_printf(&SerialA, "Motor Thetas: %.2f %.2f %.2f \n\r", printtheta1motor*180/PI, printtheta2motor*180/PI, printtheta3motor*180/PI);
        // serial_printf(&SerialA, "Position: %.2f %.2f %.2f  \n\r", x, y, z);
        // serial_printf(&SerialA, "IK DH Thetas: %.2f %.2f %.2f   \n\r", IKTh1*180/PI, IKTh2*180/PI, IKTh3*180/PI);
        // serial_printf(&SerialA, "Motor Theta (IK)s: %.2f %.2f %.2f  \n\r", IKtheta1motor*180/PI, IKtheta2motor*180/PI, IKtheta3motor*180/PI );
        // serial_printf(&SerialA, "----------------\n\r");
        serial_printf(&SerialA, "Position: %.2f %.2f %.2f  \n\r", theta_desired1, theta_desired2, theta_desired3);
    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }
}

void initialization(float t1m,float t2m) {
    /** Necessary Initialization 
     * 
     * This function performs essential setup operations required for the
     * function of the main loop.
     * 
     * updates theta1 & theta2 array with current theta motor values
     * sets blinking LEDs on Control Card and Emergency Stop Box
     * 
     * Input variables: 
     *              t1m : Theta 1 Motor [rad]
     *              t2m : Theta 2 Motor [rad]
    */

    // save past states
    if ((mycount%50)==0) {
        theta1array[arrayindex] = t1m;
        theta2array[arrayindex] = t2m;
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
}

void square_wave_trajectory(float t) {
    /** Desired theta square wave generation
     * 
     * This function creates a simple square wave trajectory.
     * If the time in seconds given by t is less than 1, we set
     * theta_desired to be pi/6. Otherwise, theta_desired is set
     * to 0.
     * 
     * Input variable: 
     *             t : time [seconds]
     */

    if (t < 1) {
        theta_desired = PI/6;
    } else {
        theta_desired = 0;
    }
}

void cubic_wave_trajectory(float t) {
    /** Desired theta cubic wave generation 
     * 
     * This function creates a cubic wave generation. 
     * We set theta_desired to be different cubic waves
     * depending on the time interval. Furthermore, feed
     * forward control is used in this instance so theta_dot
     * and theta_double_dot is also calculated for future
     * calculations.
     * 
     * Input variable: 
     *              t : time [seconds]
    */

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
}

void fun_trajectory(float t){
    /** 
     * Creating a figure 8 trajectory and solving its Inverse Kinematics
     * for the respective motor thetas.
     * 
     * We first begin by setting our x, y, and z positions at the given time
     * input t. Afterwards, we solve out our inverse kinematics in order to
     * calculate the necessary motor thetas.
     * 
     * Input variable: 
     *             t : time [seconds]
    */

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

    theta_desired1 = IKtheta1motor1;
    theta_desired2 = IKtheta2motor2;
    theta_desired3 = IKtheta3motor3;
}
