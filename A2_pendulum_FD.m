clear all; close all; clc;

%% Initialization
Tfinal = 10;   % Time range (sec)
T = 0.001;     % Sampling period (sec), 1 msec
N = Tfinal/T;  % Number of data
L=  0.7;       % Length (m)
g = 9.81;      % Gravitational Acc
m = 5;         % weight (kg)
t = linspace(0,Tfinal,N); % Time: 0~10 sec
theta=zeros(N,1);         % Angle
dq=zeros(N,1);        % Angular Vel
ddq=zeros(N,1);       % Angular Acc

%% External Torque
% a = [2,    1,    0.5,  0.3,  3]; % Magnitude
% w = [0.12, 0.33, 0.92, 1.15, 2.31]; % Frequency
% tau = a(1)*sin(2*pi*t*w(1)) + a(2)*sin(2*pi*t*w(2)) + ...
% a(3)*sin(2*pi*t*w(3)) + a(4)*sin(2*pi*t*w(4)) + a(5)*sin(2*pi*t*w(5)) ;

tau=zeros(N,1); % Free oscillation
%% Forward dynamics
theta(1) = 80*pi/180;
for k=1:N-1
    ddq(k+1) = (tau(k) - m*g*L*sin(theta(k)))/(m*L^2);
    dq(k+1) = dq(k) + ddq(k)*T; % Integration
    theta(k+1) = theta(k) + dq(k)*T;    % Integration 
end


%% Figure
figure('color','w');
theta = theta*180/pi;
subplot(211);
plot(t,theta,'b','linewidth',2); % Graph of time vs angle
ylabel('\theta (deg)'); % Name of y axis
xlabel('time(sec)'); % Name of x axis
title('Simple pendulum Inverse dynamcis')
% axis([0 10 -50 50]); % Limit of x & y axis
grid on; % Grid
set(gca,'fontsize',12); % Fontsize of the graph is 12

subplot(212);
plot(t,tau,'b','linewidth',2); % Graph of time vs angle
ylabel('\tau (Nm)'); % Name of y axis
xlabel('time(sec)'); % Name of x axis
title('Simple pendulum Inverse dynamcis')
% axis([0 10 -50 50]); % Limit of x & y axis
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
    axis([-1 1 -1 1])
    drawnow;
    hold off;
end