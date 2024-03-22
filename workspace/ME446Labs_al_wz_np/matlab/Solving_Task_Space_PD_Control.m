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