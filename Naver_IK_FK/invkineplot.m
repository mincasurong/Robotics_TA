function [P0, P1, P2, P3, P41, P42, P411, P412]=invkineplot(theta,l1,l2,l3,l4,l5,l6)



%% Initialization
theta_z1=theta(1); theta_x1=theta(2);
theta_x2=theta(3); theta_y1=theta(4);


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

%% Figure
figure('color','w');
plot3(P41(1),P41(2),P41(3),'go'); hold on; plot3([P41(1) P42(1)],[P41(2) P42(2)],[P41(3) P42(3)],'g','linewidth',2);
plot3(P411(1),P411(2),P411(3),'go'); hold on; plot3([P41(1) P411(1)],[P41(2) P411(2)],[P41(3) P411(3)],'g','linewidth',2);
plot3(P412(1),P412(2),P412(3),'go'); hold on; plot3([P42(1) P412(1)],[P42(2) P412(2)],[P42(3) P412(3)],'g','linewidth',2);
plot3(P42(1),P42(2),P42(3),'go'); hold on; 
plot3(P3(1),P3(2),P3(3),'bo'); hold on; plot3([P2(1) P3(1)],[P2(2) P3(2)],[P2(3) P3(3)],'b','linewidth',2);
plot3(P2(1),P2(2),P2(3),'co'); hold on; plot3([P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)],'c','linewidth',2);
plot3(P1(1),P1(2),P1(3),'ro'); hold on; plot3([P0(1) P1(1)],[P0(2) P1(2)],[P0(3) P1(3)],'r','linewidth',2);
plot3(P0(1),P0(2),P0(3),'k*','markersize',10); hold on;
view([1,1,1])

grid on;
axis([-10 10 -10 10 0 10]);
xlabel('x(cm)'); ylabel('y(cm)'); zlabel('z(cm)')
