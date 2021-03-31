clear all; close all; clc;

%% Initialization
Tfinal = 10; % Time range (sec)
T = 0.001; % Sampling period (sec), 1 msec
N = Tfinal/T; % Number of data
t = linspace(0,Tfinal,N); % Time: 0~10 sec
L = 0.7; % Length of pendulum is 0.3 m

%% Desired Angle
a = [20 13 12 11 5]; % Magnitude
w = [0.12 0.73 1.32 2.15 7.15]; % Frequency
theta = a(1)*sin(2*pi*t*w(1)) + a(2)*sin(2*pi*t*w(2)) + ...
a(3)*sin(2*pi*t*w(3)) + a(4)*sin(2*pi*t*w(4)) + a(5)*sin(2*pi*t*w(5)) ;
    
%% Figure

figure('color','w');
plot(t,theta,'b','linewidth',2); % Graph of time vs angle
ylabel('\theta (deg)'); % Name of y axis
xlabel('time(sec)'); % Name of x axis
title('Simple pendulum')
axis([0 10 -50 50]); % Limit of x & y axis
grid on; % Grid
set(gca,'fontsize',12); % Fontsize of the graph is 12

%% Animation
x = L*sin(theta*pi/180); y = -L*cos(theta*pi/180);
Animation_Speed = 10;

figure('color','w');
for k=1:Animation_Speed:N
    plot(x(1:k),y(1:k),'c.','linewidth',2); hold on;
    plot(x(k),y(k),'bo','markersize',30,'linewidth',2); hold on;
    plot([0 x(k)],[0 y(k)],'r','linewidth',5);
    axis([-1 1 -1 0])
    drawnow;
    hold off;
end