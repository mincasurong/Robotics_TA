clear all; close all; clc;

%% Initialization
l1=2; l2=4; l3=4; l4=1; l5=1; l6=1;
t=[180 0.001 0.001 0.001]; T=1000;
theta(:,1)=[0:(t(1)-0)/T:t(1)];
theta(:,2)=[0:(t(2)-0)/T:t(2)];
theta(:,3)=[0:(t(3)-0)/T:t(3)];
theta(:,4)=[0:(t(4)-0)/T:t(4)];
N=length(theta);
theta_x1=zeros(N,1); theta_x2=zeros(N,1); theta_z1=zeros(N,1); theta_y1=zeros(N,1);
theta_z1=theta(:,1); theta_x1=theta(:,2);
theta_x2=theta(:,3); theta_y1=theta(:,4);

P0=zeros(3,N);
P1=zeros(3,N);
P2=zeros(3,N);
P3=zeros(3,N);
P4=zeros(3,N);
P41=zeros(3,N); P411=zeros(3,N);
P42=zeros(3,N); P412=zeros(3,N);



for k=1:N,
% k=1
%% Origin(0) to Sholder-LR(1)
% z axis rotation
T01=[cos(theta_z1(k)) -sin(theta_z1(k))        0 0;
    sin(theta_z1(k))  cos(theta_z1(k))         0 0;
    0             0                      1 l1;
    0             0                      0 1];

%% Sholder-LR(1) to Sholder-FB(2)

T11=[-1        0              0             0;
    0         cos(theta_x1(k))  -sin(theta_x1(k)) 0;
    0         sin(theta_x1(k))  cos(theta_x1(k))  0;
    0         0              0              1];

T12=[-1        0              0              0;
    0         cos(theta_x2(k))  -sin(theta_x2(k)) l2;
    0         sin(theta_x2(k))  cos(theta_x2(k))  0;
    0         0              0              1];


%% Sholder-FB(2) to Elbow(3)
T23=[1 0 0 0;
     0 1 0 l3;
     0 0 1 0;
     0 0 0 1];
 

%% Elbow(3) to End effector(4)

T34=[cos(theta_y1(k))         0        sin(theta_y1(k)) 0;
     0                     1        0             0;
     -sin(theta_y1(k))        0        cos(theta_y1(k)) 0;
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

P0(:,k)=[0;0;0];
P1(:,k)=T1(1:3,4);
P2(:,k)=T2(1:3,4);
P3(:,k)=T3(1:3,4);
P4(:,k)=T4(1:3,4);

P41(:,k)=T41(1:3,4); P411(:,k)=T411(1:3,4);
P42(:,k)=T42(1:3,4); P412(:,k)=T421(1:3,4);

end

%% Figure
figure('color','w');
for k=1:N,
plot3(P41(1,k),P41(2,k),P41(3,k),'go'); hold on; plot3([P41(1,k) P42(1,k)],[P41(2,k) P42(2,k)],[P41(3,k) P42(3,k)],'g','linewidth',2);
plot3(P411(1,k),P411(2,k),P411(3,k),'go'); hold on; plot3([P41(1,k) P411(1,k)],[P41(2,k) P411(2,k)],[P41(3,k) P411(3,k)],'g','linewidth',2);
plot3(P412(1,k),P412(2,k),P412(3,k),'go'); hold on; plot3([P42(1,k) P412(1,k)],[P42(2,k) P412(2,k)],[P42(3,k) P412(3,k)],'g','linewidth',2);
plot3(P42(1,k),P42(2,k),P42(3,k),'go'); hold on; 
plot3(P3(1,k),P3(2,k),P3(3,k),'bo'); hold on; plot3([P2(1,k) P3(1,k)],[P2(2,k) P3(2,k)],[P2(3,k) P3(3,k)],'b','linewidth',2);
plot3(P2(1,k),P2(2,k),P2(3,k),'co'); hold on; plot3([P1(1,k) P2(1,k)],[P1(2,k) P2(2,k)],[P1(3,k) P2(3,k)],'c','linewidth',2);
plot3(P1(1,k),P1(2,k),P1(3,k),'ro'); hold on; plot3([P0(1,k) P1(1,k)],[P0(2,k) P1(2,k)],[P0(3,k) P1(3,k)],'r','linewidth',2);
plot3(P0(1,k),P0(2,k),P0(3,k),'k*','markersize',10); hold on;
plot3(P3(1,1:k),P3(2,1:k),P3(3,1:k),'b.'); hold on; 
view([1,1,1])

grid on;
axis([-10 10 -10 10 -10 10]);
hold off;
drawnow;
end
xlabel('x(cm)'); ylabel('y(cm)'); zlabel('z(cm)')