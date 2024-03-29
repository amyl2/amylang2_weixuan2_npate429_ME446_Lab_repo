syms thetaM3 thetaM2 thetaM1 xde yde zde x y z xdd ydd zdd xd yd zd KPx KPy KPz KDx KDy KDz
syms JT_11 JT_12 JT_13 JT_21 JT_22 JT_23 JT_31 JT_32 JT_33

% J_t = [-0.254*sin(thetaM1)*(cos(thetaM3)+sin(thetaM2)), 0.254*cos(thetaM1)*(cos(thetaM3)+sin(thetaM2)) , 0;
%        0.254*cos(thetaM1)*(cos(thetaM2)-sin(thetaM3)), 0.254*sin(thetaM1)*(cos(thetaM2)-sin(thetaM3)), -0.254*(cos(thetaM3)+sin(thetaM2));
%        -0.254*cos(thetaM1)*sin(thetaM3), -0.254*sin(thetaM1)*sin(thetaM3), -0.254*cos(thetaM3)
%        ]

JT = [ JT_11 JT_12 JT_13; 
       JT_21 JT_22 JT_23;
       JT_31 JT_32 JT_33];

wut = [KPx*(xde-x) + KDx*(xdd-xd);
       KPy*(yde-y) + KDy*(ydd-yd);
       KPz*(zde-z) + KDz*(zdd-zd)];

JT*wut

syms Xcmd Ycmd Zcmd

x = [0; 0; Zcmd];
JT*x

syms tx ty tz

R1 = [cos(tz), -sin(tz), 0;
      sin(tz), cos(tz), 0;
      0, 0, 1];
R2 = [1, 0, 0;
      0, cos(tx), -sin(tx);
      0 sin(tx), cos(tx)];
R3 = [cos(ty), 0, sin(ty);
      0, 1, 0;
      -sin(ty), 0, cos(ty)];

RWN = R1*R2*R3;

KP = [KPx 0 0;
      0 KPy 0;
      0 0 KPz];
KD = [KDx 0 0;
      0 KDy 0;
      0 0 KDz];
RNW = RWN.';
syms x_error y_error z_error xd_error yd_error zd_error
error = [x_error; y_error; z_error];
error_dot = [xd_error; yd_error; zd_error];
tau = JT*RWN*(KP*RNW*error + KD*RNW*error_dot);
simplify(tau)

