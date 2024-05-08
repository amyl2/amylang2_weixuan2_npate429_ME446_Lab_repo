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

    Forwards_Kinematics(theta1motor,theta2motor,theta3motor);

    /** Creating desired waves
         *
         * This section deals with generating trajectories that the CRS
         * Robot Arm will follow during its operation.
         *
         * All trajectories are straight lines. Depending on current time, different
         * segments of the straight line are used as the reference trajectory.
         *
         * 
        */

    /** Final Project Trajectories */
    float t_waypoint = mycount/1000.0;
    if ((t_waypoint < wp2.time)) {
        StraightLine(wp1, wp2); // Straight Line Out
    } else if ((t_waypoint < wp3.time)){
        StraightLine(wp2, wp3); // Center over peg
    } else if ((t_waypoint < wp4.time)) {
        StraightLine(wp3, wp4); // "" but slightly lower
    } else if (t_waypoint < wp5.time){
        // Turn off impendance control in both x and y directions
        // This allows the end effector to move freely in those directions
        // so that it can slide into the hole guided by the countersink
        Set_Imp_Gains(0,0,KPz,0,0,KDz, false);
        StraightLine(wp4, wp5); // "" but EVEN lower
    } else if (t_waypoint < wp6.time){
        StraightLine(wp5,wp6); // Hold
    } else if (t_waypoint < wp7.time){
        Set_Imp_Gains(0,0,KPz,0,0,KDz, true); // Reset Gains
        StraightLine(wp6,wp7); // Rise above
    } else if (t_waypoint < wp8.time){
        StraightLine(wp7,wp8); // Above maze
    } else if (t_waypoint < wp9.time){
        StraightLine(wp8,wp9); // At maze
    } else if (t_waypoint < wp10.time){
        // The impedance control in original y axis is turned off
        // the original coordinate frame is rotated around original z axis by -53 degrees
        // This allows end effector to move freely in directions perpendicular to the maze line
        tz = wp10.theta_z;
        Set_Imp_Gains(KPx,0,KPz,KDx,0,KDz, false); // Turn off y gains
        StraightLine(wp9,wp10);
    } else if (t_waypoint < wp11.time){
        // first transition point after the first maze line
        // move in the y direction and allow to move freely in x direction
        // the end-effector is expected to stay near the outer maze wall
        tz = wp11.theta_z;
        Set_Imp_Gains(0,KPy,KPz,KDx,0,KDz, true);
        Set_Imp_Gains(0,600,KPz,0,40,KDz, false); 
        StraightLine(wp10,wp11); 
    } else if (t_waypoint < wp12.time){
        // second transition point after the first maze line
        // move in top-left direction, sliding along the outer maze wall
        tz = wp12.theta_z;
        Set_Imp_Gains(KPx,0,KPz,KDx,0,KDz, true);
        Set_Imp_Gains(0,600,KPz,0,40,KDz, false);
        StraightLine(wp11,wp12);
    } else if (t_waypoint < wp13.time){
        // third transition point after the first maze line
        // continue to move in top-left direction, but with a larger up (-x) component
        tz = wp13.theta_z;
        Set_Imp_Gains(KPx,0,KPz,KDx,0,KDz, true); 
        Set_Imp_Gains(600,0,KPz,40,0,KDz, false);
        StraightLine(wp12,wp13);
    } else if (t_waypoint < wp14.time){
        // Second maze line
        // Turn off gains in y direction and rotate around original z by -15 deg
        // allows movement perpendicular to the second maze line
        tz = wp14.theta_z;
        Set_Imp_Gains(KPx,0,KPz,KDx,0,KDz, true);
        Set_Imp_Gains(600,0,KPz,40,0,KDz, false);
        StraightLine(wp13,wp14);
    } else if (t_waypoint < wp15.time){
        // first transition point after the second maze line
        // Move a little more in the direction of the second maze line
        tz = wp15.theta_z;
        Set_Imp_Gains(KPx,0,KPz,KDx,0,KDz, true);
        Set_Imp_Gains(600,0,KPz,40,0,KDz, false);
        StraightLine(wp14,wp15);
    } else if (t_waypoint < wp16.time){
        // second transition point after the second maze line
        // move in top-left direction
        tz = wp16.theta_z;
        Set_Imp_Gains(KPx,0,KPz,KDx,0,KDz, true);
        Set_Imp_Gains(0,800,KPz,0,60,KDz, false);
        StraightLine(wp15,wp16);
    } else if (t_waypoint < wp17.time){
        // third transition point after the second maze line
        // move in down-left direction
        tz = wp17.theta_z;
        Set_Imp_Gains(KPx,0,KPz,KDx,0,KDz, true);
        Set_Imp_Gains(0,400,KPz,0,20,KDz, false);
        StraightLine(wp16,wp17);
    } else if (t_waypoint < wp18.time){
        // last maze line
        tz = wp18.theta_z;
        Set_Imp_Gains(KPx,0,KPz,KDx,0,KDz, true);
        Set_Imp_Gains(300,0,KPz,20,0,KDz, false); 
        StraightLine(wp17,wp18);
    } else if (t_waypoint < wp19.time){
        // rise above from the maze exit
        Set_Imp_Gains(KPx,0,KPz,KDx,0,KDz, true);
        Set_Imp_Gains(800,600,KPz,20,15,KDz, false);
        StraightLine(wp18,wp19);
    } else if (t_waypoint < wp20.time){
        // move horizontally to above the egg position
        // gains are high to ensure end effector height tracking is accurate
        Set_Imp_Gains(KPx,0,KPz,KDx,0,KDz, true);
        Set_Imp_Gains(800,600,800,20,20,20, false);
        StraightLine(wp19,wp20);
    }  else if (t_waypoint < wp21.time){
        // push down egg
        StraightLine(wp20,wp21);
    } else if (t_waypoint < wp22.time){
        // hold the push down position
        // wp21 and wp22 have the same positions, but different arrival times
        StraightLine(wp21,wp22);
    } else if (t_waypoint < wp23.time){
        // go to zero position as defined by zero D-H angles
        Set_Imp_Gains(KPx,0,KPz,KDx,0,KDz, true); // reset Imp_Gains
        StraightLine(wp22,wp23); 
    } else {
        // hold at zero position
        Set_Imp_Gains(KPx,0,KPz,KDx,0,KDz, true); 
        Hold(wp23);
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

    /** 
         *
         * Calculates the desired x, y, z position (in m) and x, y, z velocity (in m/s) in the current instant
         * such that the end effector moves from waypoing "init" to "final" in a straight line at a constant
         * velocity, as defined by delta_x / t_total 
         * 
         * This function is expected to be called in every cycle to update the desired x, y, z position
         * 
         * Note: the current time (seconds) as calculated by mycount/1000.0 should be within [init.time, final.time]
         * to generate the desired straight line.
         * 
         * Input:
         * init: start waypoint
         * final: end waypoint
         * each waypoing contains: x, y, z position (m) of the waypoint; time (s) counting from the start of motion at which
         * the waypoint is reached.
        */

    float t_total_1 = fabs(final.time - init.time);
    float delta_x = final.xDes - init.xDes;
    float delta_y = final.yDes - init.yDes;
    float delta_z = final.zDes - init.zDes;

    xde = (delta_x)*(mycount/1000.0 - init.time)/t_total_1 + init.xDes;
    yde = (delta_y)*(mycount/1000.0 - init.time)/t_total_1 + init.yDes;
    zde = (delta_z)*(mycount/1000.0 - init.time)/t_total_1 + init.zDes;

    xdd = (delta_x)/t_total_1;
    ydd = (delta_y)/t_total_1;
    zdd = (delta_z)/t_total_1;

}

void Hold(Waypoint hold_point) {

    /** 
         *
         * Calculates the desired x, y, z position (in m) and x, y, z velocity (in m/s) in the current instant
         * such that the end effector remains stationary at hold_point
         * All velocities are set to 0, and position is set to that of the waypoint
         * 
        */  
    xde = hold_point.xDes;
    yde = hold_point.yDes;
    zde = hold_point.zDes;
    xdd = 0;
    ydd = 0;
    zdd = 0;
}

void Set_Imp_Gains(float KPx_new, float KPy_new, float KPz_new, float KDx_new, float KDy_new, float KDz_new, bool reset) {
    /** 
         *
         * Set the proportional, derivative gains of PD controller in impedance control
         * x, y, z refer to the world frame fixed at the base of the robot
         * If reset is true, all gains are set to hard-coded default values, no matter what other inputs are
         * If reset is false, the gains are set according to inputs
         * 
         * Input:
         * KPx_new, KPy_new, KPz_new: new proportional gains in x, y, and z directions
         * KDx_new, KDy_new, KDz_new: new derivative gains in x, y, and z directions
         * reset: whether or not to set all gains to default values
         * 
        */
    if (reset) {
        KPx = 280.0; // 280.0
        KPy = 150.0; // 150.0
        KPz = 220.0; // 220.0

        // KD gains for x, y, and z
        KDx = 19.0; // 19.0
        KDy = 12.0; //12.0
        KDz = 12.0; //12.0
    } else {
        KPx = KPx_new;
        KPy = KPy_new;
        KPz = KPz_new;
        KDx = KDx_new;
        KDy = KDy_new;
        KDz = KDz_new;
    }
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
        // Print actual (x, y, z) and desired (x, y, z)
         serial_printf(&SerialA, "Position: %.3f %.3f %.3f %.3f %.3f %.3f \n\r", x, y, z , xde, yde, zde);
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
