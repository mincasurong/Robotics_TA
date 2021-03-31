clear all; close all; clc;

%% Initialization

syms l1 l2 l3 l4 l5 l6 theta_z1 theta_x1 theta_x2 theta_y1

%% Origin(0) to Sholder-LR(1)
% z axis rotation
T01=[cos(theta_z1) -sin(theta_z1)        0 0;
    sin(theta_z1)  cos(theta_z1)         0 0;
    0             0                      1 l1;
    0             0                      0 1];

%% Sholder-LR(1) to Sholder-FB(2)

T11=[-1        0              0             0;
    0         cos(theta_x1)  -sin(theta_x1) 0;
    0         sin(theta_x1)  cos(theta_x1)  0;
    0         0              0              1];

T12=[-1        0              0              0;
    0         cos(theta_x2)  -sin(theta_x2) l2;
    0         sin(theta_x2)  cos(theta_x2)  0;
    0         0              0              1];


%% Sholder-FB(2) to Elbow(3)
T23=[1 0 0 0;
    0 1 0 l3;
    0 0 1 0;
    0 0 0 1];


%% Elbow(3) to End effector(4)

T34=[cos(theta_y1)         0        sin(theta_y1) 0;
    0                     1        0             0;
    -sin(theta_y1)        0        cos(theta_y1) 0;
    0                     0        0             1];

 %% Finger
T411= [1 0 0 0;
      0 1 0 0;
      0 0 1 l5;
      0 0 0 1];
T412= [1 0 0 0;
      0 1 0 0;
      0 0 1 -l5;
      0 0 0 1];
T441=[1 0 0 0;
      0 1 0 l6;
      0 0 1 0;
      0 0 0 1];
T442=[1 0 0 0;
      0 1 0 l6;
      0 0 1 0;
      0 0 0 1];
R4411=[-1        0              0     0;
    0         cos(-pi/2)  -sin(-pi/2) 0;
    0         sin(-pi/2)  cos(-pi/2)  0;
    0         0              0      1];
R4421=[-1        0              0     0;
    0         cos(pi/2)  -sin(pi/2) 0;
    0         sin(pi/2)  cos(pi/2)  0;
    0         0              0      1];  



%% Position
T1=T01;
T2=T1*(T11*T12);
T3=T2*T23;
T4=T3*T34;
T41=T4*T411; 
T411=T41*T441;
T42=T4*T412; 
T421=T42*T442;

P0=[0;0;0;1];
P1=T1(1:3,4);
P2=T2(1:3,4);
P3=T3(1:3,4);
P4=T4(1:3,4);

P41=T41(1:3,4); P411=T411(1:3,4);
P42=T42(1:3,4); P412=T421(1:3,4);

%% Jacobian

Px=T41(1,4); Py=T41(2,4); Pz=T41(3,4);

J=simplify([diff(Px,theta_x1) diff(Px,theta_x2) diff(Px,theta_y1) diff(Px,theta_z1);
    diff(Py,theta_x1) diff(Py,theta_x2) diff(Py,theta_y1) diff(Py,theta_z1);
    diff(Pz,theta_x1) diff(Pz,theta_x2) diff(Pz,theta_y1) diff(Pz,theta_z1)]);

J=@(l1,l2,l3,l4,l5,theta_x1,theta_x2,theta_y1,theta_z1)...
[   l3*sin(theta_z1)*(cos(theta_x1)*sin(theta_x2) + cos(theta_x2)*sin(theta_x1)) + l2*sin(theta_x1)*sin(theta_z1) + l5*cos(theta_y1)*sin(theta_z1)*(cos(theta_x1)*cos(theta_x2) - sin(theta_x1)*sin(theta_x2)),  sin(theta_z1)*(l3*sin(theta_x1 + theta_x2) + l5*cos(theta_x1 + theta_x2)*cos(theta_y1)), l5*(cos(theta_y1)*cos(theta_z1) - sin(theta_y1)*sin(theta_z1)*(cos(theta_x1)*sin(theta_x2) + cos(theta_x2)*sin(theta_x1))), - l5*(sin(theta_y1)*sin(theta_z1) - cos(theta_y1)*cos(theta_z1)*(cos(theta_x1)*sin(theta_x2) + cos(theta_x2)*sin(theta_x1))) - l3*cos(theta_z1)*(cos(theta_x1)*cos(theta_x2) - sin(theta_x1)*sin(theta_x2)) - l2*cos(theta_x1)*cos(theta_z1);
  - l3*cos(theta_z1)*(cos(theta_x1)*sin(theta_x2) + cos(theta_x2)*sin(theta_x1)) - l2*cos(theta_z1)*sin(theta_x1) - l5*cos(theta_y1)*cos(theta_z1)*(cos(theta_x1)*cos(theta_x2) - sin(theta_x1)*sin(theta_x2)), -cos(theta_z1)*(l3*sin(theta_x1 + theta_x2) + l5*cos(theta_x1 + theta_x2)*cos(theta_y1)), l5*(cos(theta_y1)*sin(theta_z1) + cos(theta_z1)*sin(theta_y1)*(cos(theta_x1)*sin(theta_x2) + cos(theta_x2)*sin(theta_x1))),   l5*(cos(theta_z1)*sin(theta_y1) + cos(theta_y1)*sin(theta_z1)*(cos(theta_x1)*sin(theta_x2) + cos(theta_x2)*sin(theta_x1))) - l3*sin(theta_z1)*(cos(theta_x1)*cos(theta_x2) - sin(theta_x1)*sin(theta_x2)) - l2*cos(theta_x1)*sin(theta_z1);
  l3*cos(theta_x1 + theta_x2) + l2*cos(theta_x1) - l5*sin(theta_x1 + theta_x2)*cos(theta_y1),                  l3*cos(theta_x1 + theta_x2) - l5*sin(theta_x1 + theta_x2)*cos(theta_y1),                                                                                 -l5*cos(theta_x1 + theta_x2)*sin(theta_y1),                                                                                                                                                                                                                                            0]
 

l1=2; l2=4; l3=4; l4=1; l5=1; l6=1;
thetaset=[10 180 180 60]*pi/180; % z1 x1 x2 z1
theta_z1=thetaset(1); theta_x1=thetaset(2); % Initial theta
theta_x2=thetaset(3); theta_y1=thetaset(4);
Jvalue=J(l1,l2,l3,l4,l5,theta_x1,theta_x2,theta_y1,theta_z1);
invJacobian=inv(Jvalue'*Jvalue)*Jvalue' % Inverse

xdot=[3; -10; 100];
thetadot=invJacobian*xdot;
alpha=1;
theta=alpha*thetadot;
theta*180/pi

%% Figure
[P0, P1, P2, P3, P41, P42, P411, P412]=invkineplot(theta,l1,l2,l3,l4,l5,l6);