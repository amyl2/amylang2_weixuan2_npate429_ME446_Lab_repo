#include "math.h"
#include <stdbool.h>
#include "F28335Serial.h"
#include "Final_Project.h"

/** ********** Main ********** **/
void main(void)
{
    mains_code();
}

// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3) {
    /** necessary initialization */
    initialization(theta1motor, theta2motor);

    /** Creating desired waves
         *
         * This section deals with generating trajectories or desired waveforms that the CRS
         * Robot Arm will follow during its operation.
         *
         * Toggle commenting statements in order to choose the trajectory to follow!
         * By uncommenting or commenting specific lines of code, you can select different
         * trajectory profiles such as step,or straight line.
         *
         * Note: Only one trajectory should be uncommented at all times to ensure that the
         * robot arm follows the intended path. Mixing multiple trajectories or waveforms
         * can lead to unpredictable or undesired motion.
        */

    /* Desired line wave generation */
    // Period of the line is (2 * t_total): t_total seconds from xb to xa, t_total seconds back
//    if ((mycount%(2000*t_total)) < 1000.0*t_total) {
//        xde = (-delta_x*(mycount%(2000*t_total)))/(t_total*1000.0) + xb;
//        yde = (-delta_y*(mycount%(2000*t_total)))/(t_total*1000.0) + yb;
//        zde = (-delta_z*(mycount%(2000*t_total)))/(t_total*1000.0) + zb;
//    } else {
//        xde = (delta_x*(mycount%(2000*t_total - 1000*t_total)))/(t_total*1000.0) + xa;
//        yde = (delta_y*(mycount%(2000*t_total - 1000*t_total)))/(t_total*1000.0) + ya;
//        zde = (delta_z*(mycount%(2000*t_total - 1000*t_total)))/(t_total*1000.0) + za;
//    }

    /** Final Project Trajectories */

    if ((mycount/1000.0 < 2)) {
        StraightLine(wp1, wp2);
    } else if ((mycount/1000.0 < 4)){
        StraightLine(wp2, wp3);
    } else if ((mycount/1000.0 < 6)) {
        StraightLine(wp3, wp4);
    } else {
        Hold(wp4);
    }

    /** Code to make it work :) */

    // Functions for Controlling Motor
    Infinite_Impulse_Response(theta1motor,theta2motor,theta3motor);
    Rot_FF_IC_calc(theta1motor,theta2motor,theta3motor);
    Friction();
    Motor_Control(tau1,tau2,tau3);

    // To send to simulink & print variables
    Output_Vars(theta1motor,theta2motor,theta3motor);

    // Cleaning up variables for next iteration
    Updating_Vars(theta1motor,theta2motor,theta3motor);
    Forwards_Kinematics(theta1motor,theta2motor,theta3motor);
    Inverse_Kinematics();
}

void StraightLine(Waypoint init, Waypoint final) {

//    float max_velocity = 5; // m/s

    float t_total_1 = fabs(final.time - init.time);
    float delta_x = final.xDes - init.xDes;
    float delta_y = final.yDes - init.yDes;
    float delta_z = final.zDes - init.zDes;
    float norm = pow(pow(delta_x, 2) + pow(delta_y,2) + pow(delta_z,2), 0.5);

    xde = (delta_x)*(mycount/1000.0 - init.time)/t_total_1 + init.xDes;
    yde = (delta_y)*(mycount/1000.0 - init.time)/t_total_1 + init.yDes;
    zde = (delta_z)*(mycount/1000.0 - init.time)/t_total_1 + init.zDes;

    xdd = (delta_x)/t_total_1;
    ydd = (delta_y)/t_total_1;
    zdd = (delta_z)/t_total_1;

//    float max_velx = max_velocity*delta_x/norm;
//    float max_vely = max_velocity*delta_y/norm;
//    float max_velz = max_velocity*delta_z/norm;
//
//    xdd = -max_velx/(pow(t_total_1/2,2)) * pow((mycount/1000.0 - t_total_1/2),2) + max_velx;
//    ydd = -max_vely/(pow(t_total_1/2,2)) * pow((mycount/1000.0 - t_total_1/2),2) + max_vely;
//    zdd = -max_velz/(pow(t_total_1/2,2)) * pow((mycount/1000.0 - t_total_1/2),2) + max_velz;
}

void Hold(Waypoint hold_point) {
    xde = hold_point.xDes;
    yde = hold_point.yDes;
    zde = hold_point.zDes;
    xdd = 0;
    ydd = 0;
    zdd = 0;
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
         serial_printf(&SerialA, "Position: %.2f %.2f %.2f %.2f %.2f %.2f \n\r", x, y, z , xde, yde, zde);
        // serial_printf(&SerialA, "IK DH Thetas: %.2f %.2f %.2f   \n\r", IKTh1*180/PI, IKTh2*180/PI, IKTh3*180/PI);
        // serial_printf(&SerialA, "Motor Theta (IK)s: %.2f %.2f %.2f  \n\r", IKtheta1motor*180/PI, IKtheta2motor*180/PI, IKtheta3motor*180/PI );
        // serial_printf(&SerialA, "----------------\n\r");
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

void Infinite_Impulse_Response(float theta1motor,float theta2motor,float theta3motor) {
    /** Infinite Impulse Response
     *
     * The first section implements the Infinite Impulse Response (IIR) filtering technique
     * to calculate raw angular velocity (Omega) optimally. The second section implements the IIR
     * filter for the x, y, and z coordinate system velocities.
     *
     * Omega, representing angular velocity, is first found by using a simple difference
     * in thetas equation for a basic estimation. Afterwards, an Infinite Impulse Response (IIR)
     * averaging filter is used to filer and smoothen the omega calculation. On the second section,
     * similar calculations are done for the different coordinate system.
     *
     * This ensures that the values used for later control calculations are more
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

    // First simple x_dot calculations using theta differences.
    xd = (x - x_old)/0.001;
    yd = (y - y_old)/0.001;
    zd = (z - z_old)/0.001;

    // implementing averaging filter for velocities
    xd = (xd + xd_old1 + xd_old2)/3.0;
    yd = (yd + yd_old1 + yd_old2)/3.0;
    zd = (zd + zd_old1 + zd_old2)/3.0;
}

void Rot_FF_IC_calc(float theta1motor,float theta2motor,float theta3motor) {
    /** Rotation, Feed Forward, and Impedance Control Calculations
     *
     * Feed Forward forces are applied to have the arm behave in certain manners. This was
     * first done in the Z direction where the Kp and Kd gains in the z direction were set
     * to zero, which resulted in the arm being easily moved in the z direction, but not in
     * the x or y directions.
     *
     * Impedance control can be used to alter the stiffness and PD response of the arm
     * along certain axes. For example, by reducing the Kd and Kp gains along the x-axis,
     * you can weaken or completely remove resistance in that direction.
     *
     * We want to be able to control impedance along any axis however, so we can apply a
     * set of three rotations about the world x, y, and z axis. These rotations can be done
     * by different amounts to change the weakened axis to any desired axis.
    */

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

    // calculations for impedance control
    ctx = cos(tx);
    cty = cos(ty);
    ctz = cos(tz);
    stx = sin(tx);
    sty = sin(ty);
    stz = sin(tz);

    // calculating position and velocity errors
    x_error = xde-x;
    y_error = yde-y;
    z_error = zde-z;

    xd_error = xdd-xd;
    yd_error = ydd-yd;
    zd_error = zdd-zd;

    // Jacobian Transpose calculations
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
}

void Friction() {
    /** Friction
         *
         * Compensating for friction means that the arm moves in a more smooth, free fashion, and is thus
         * useful when programming the robot to perform motions. The friction compensation was done using
         * the friction plots provided in the lab manual for the CRS robot arm after testing. With friction
         * compensation, the robot arm moves easily, but it does not accelerate or bounce around when pushed.
         *
        */

        // Joint 1 calculating friction
        // case 1: |omega| < min_velocity: static friction, approximated by a line with high slope
        // case 2 and 3: |omega| >= min_velocity: sliding friction, approximated by a line with low slope
        // The switching point between different cases are continuous to ensure smooth torque
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
        // underestimation is preferred, as in the case of overestimation the robot is unstable
        u_fric1 = u_fric1*fric_fac1;
        u_fric2 = u_fric2*fric_fac2;
        u_fric3 = u_fric3*fric_fac3;
}


void Motor_Control(float *tau1,float *tau2,float *tau3) {
    /** Motor Control
         *
         *
         * local variables:
         *        - fric_test          : Boolean for desired friction cancellation test
         *        - TS_Feed_Forwards   : Boolean for desired Feed Forwards test
         *        - impedance_control  : Boolean for desired Impedance Control test
         *
         * Note: Only one test should be true at all times to ensure that the
         * robot arm follows the intended path. Not doing so may result in undesired behavior.
        */

        // Zcmd is used to offset gravity and apply/cancel an external force if required
        Zcmd = Zcmd_offset + Zcmd_force;

        bool fric_test = false;
        bool TS_Feed_Forwards = false;
        bool impedance_control = true;

        if (fric_test) {
            // Only add friction compensation
            // Arm is expected to move by hand easily
            *tau1 = u_fric1;
            *tau2 = u_fric2;
            *tau3 = u_fric3;
        } else if (TS_Feed_Forwards) {
            // The control law is u = JT * (Kp * error + Kd * error_dot) + fric_compensation
            // Arm is expected to follow a trajectory specified by (xde, yde, zde), (xdd, ydd, zdd)
            *tau1 = -JT_11*(KPx*(x - xde) + KDx*(xd - xdd)) - JT_12*(KPy*(y - yde) + KDy*(yd - ydd)) - JT_13*(KPz*(z - zde) + KDz*(zd - zdd)) + u_fric1 + JT_13*Zcmd/Kt;
            *tau2 =  -JT_21*(KPx*(x - xde) + KDx*(xd - xdd)) - JT_22*(KPy*(y - yde) + KDy*(yd - ydd)) - JT_23*(KPz*(z - zde) + KDz*(zd - zdd)) + u_fric2 + JT_23*Zcmd/Kt;
            *tau3 = -JT_31*(KPx*(x - xde) + KDx*(xd - xdd)) - JT_32*(KPy*(y - yde) + KDy*(yd - ydd)) - JT_33*(KPz*(z - zde) + KDz*(zd - zdd)) + u_fric3 + JT_33*Zcmd/Kt;
        } else if (impedance_control) {
            // The control law aims to make the end-effector act like a mass spring damper system
            *tau1 = (JT_11*(ctz*sty + cty*stx*stz) + JT_12*(sty*stz - cty*ctz*stx) + JT_13*ctx*cty)*(KDz*xd_error*(ctz*sty + cty*stx*stz) + KPz*x_error*(ctz*sty + cty*stx*stz) + KDz*yd_error*(sty*stz - cty*ctz*stx) + KPz*y_error*(sty*stz - cty*ctz*stx) + KDz*zd_error*ctx*cty + KPz*z_error*ctx*cty) + (JT_11*(cty*ctz - stx*sty*stz) + JT_12*(cty*stz + ctz*stx*sty) - JT_13*ctx*sty)*(KDx*xd_error*(cty*ctz - stx*sty*stz) + KPx*x_error*(cty*ctz - stx*sty*stz) + KDx*yd_error*(cty*stz + ctz*stx*sty) + KPx*y_error*(cty*stz + ctz*stx*sty) - KDx*zd_error*ctx*sty - KPx*z_error*ctx*sty) + (JT_13*stx + JT_12*ctx*ctz - JT_11*ctx*stz)*(KDy*zd_error*stx + KPy*z_error*stx + KDy*yd_error*ctx*ctz + KPy*y_error*ctx*ctz - KDy*xd_error*ctx*stz - KPy*x_error*ctx*stz) + u_fric1;
            *tau2 = (JT_21*(ctz*sty + cty*stx*stz) + JT_22*(sty*stz - cty*ctz*stx) + JT_23*ctx*cty)*(KDz*xd_error*(ctz*sty + cty*stx*stz) + KPz*x_error*(ctz*sty + cty*stx*stz) + KDz*yd_error*(sty*stz - cty*ctz*stx) + KPz*y_error*(sty*stz - cty*ctz*stx) + KDz*zd_error*ctx*cty + KPz*z_error*ctx*cty) + (JT_21*(cty*ctz - stx*sty*stz) + JT_22*(cty*stz + ctz*stx*sty) - JT_23*ctx*sty)*(KDx*xd_error*(cty*ctz - stx*sty*stz) + KPx*x_error*(cty*ctz - stx*sty*stz) + KDx*yd_error*(cty*stz + ctz*stx*sty) + KPx*y_error*(cty*stz + ctz*stx*sty) - KDx*zd_error*ctx*sty - KPx*z_error*ctx*sty) + (JT_23*stx + JT_22*ctx*ctz - JT_21*ctx*stz)*(KDy*zd_error*stx + KPy*z_error*stx + KDy*yd_error*ctx*ctz + KPy*y_error*ctx*ctz - KDy*xd_error*ctx*stz - KPy*x_error*ctx*stz) + u_fric2;
            *tau3 = (JT_31*(ctz*sty + cty*stx*stz) + JT_32*(sty*stz - cty*ctz*stx) + JT_33*ctx*cty)*(KDz*xd_error*(ctz*sty + cty*stx*stz) + KPz*x_error*(ctz*sty + cty*stx*stz) + KDz*yd_error*(sty*stz - cty*ctz*stx) + KPz*y_error*(sty*stz - cty*ctz*stx) + KDz*zd_error*ctx*cty + KPz*z_error*ctx*cty) + (JT_31*(cty*ctz - stx*sty*stz) + JT_32*(cty*stz + ctz*stx*sty) - JT_33*ctx*sty)*(KDx*xd_error*(cty*ctz - stx*sty*stz) + KPx*x_error*(cty*ctz - stx*sty*stz) + KDx*yd_error*(cty*stz + ctz*stx*sty) + KPx*y_error*(cty*stz + ctz*stx*sty) - KDx*zd_error*ctx*sty - KPx*z_error*ctx*sty) + (JT_33*stx + JT_32*ctx*ctz - JT_31*ctx*stz)*(KDy*zd_error*stx + KPy*z_error*stx + KDy*yd_error*ctx*ctz + KPy*y_error*ctx*ctz - KDy*xd_error*ctx*stz - KPy*x_error*ctx*stz) + u_fric3;
        }

        // Saturate the torque to safe values
        Torque_Saturation(tau1,tau2,tau3);
}

void Torque_Saturation(float *tau1,float *tau2,float *tau3) {
    /** Saturate the Torque Values
         *
         * Because all real motors have limited torque capability,
         * it is important to saturate the torque values to prevent
         * excessive stress on the motors and maintain safe operation
         * of the robot.
         *
         * This snippet caps the torque value at +/- 5.
         */

        /* Motor 1 */
        if (*tau1 > 5.0){
            *tau1 = 5.0;
        } else if ( *tau1 < -5.0 ){
            *tau1 = -5.0;
        }

        /* Motor 2 */
        if (*tau2 > 5.0){
            *tau2 = 5.0;
        } else if ( *tau2 < -5.0 ){
            *tau2 = -5.0;
        }

        /* Motor 3 */
        if (*tau3 > 5.0){
            *tau3 = 5.0;
        } else if ( *tau3 < -5.0 ){
            *tau3 = -5.0;
        }
}

void Updating_Vars(float theta1motor,float theta2motor,float theta3motor) {
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

    // for x_dot
    x_old = x;
    xd_old2 = xd_old1;
    xd_old1 = xd;

    // for y_dot
    y_old = y;
    yd_old2 = yd_old1;
    yd_old1 = yd;

    // for z_dot
    z_old = z;
    zd_old2 = zd_old1;
    zd_old1 = zd;

    // for overall loop
    mycount++;
}

void Forwards_Kinematics(float theta1motor,float theta2motor,float theta3motor) {
    /** Forward Kinematics (simulation)
     *
     * This code is used to solve out the forwards kinematics
     * in order to calculate x, y, and z, and it could also be used to
     * check that the positions and DH angles of the end-effector are as expected.
     *
     * We define x,y,z positions [meters] as well as the DH theta values [rad]
     * based on our previous calculations in lab 1.
    */
    x = 0.254*cos(theta1motor)*(cos(theta3motor)+sin(theta2motor));
    y = 0.254*sin(theta1motor)*(cos(theta3motor)+sin(theta2motor));
    z = 0.254*cos(theta2motor)-0.254*sin(theta3motor)+0.254;

    Th2DH = theta2motor - PI/2;
    Th3DH = -theta2motor + theta3motor + PI/2;
}

void Inverse_Kinematics() {
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
}

void Output_Vars(float theta1motor, float theta2motor, float theta3motor) {
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
    printtheta1motor = x;
    printtheta2motor = y;
    printtheta3motor = z;

    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
    Simulink_PlotVar4 = theta1motor;
}