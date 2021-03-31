clear all; close all; clc;

% Forward kinematics
%% 시뮬레이션 파라미터

T = 0.001; % Sampling period [sec] 
Tfinal = 2; % 시뮬레이션 최종 시간 [sec] 
t = 0:T:Tfinal; % 시뮬레이션 시간 정의 [sec]
N = length(t); % 시뮬레이션 Sample 수

L1 = 0.41; L2 = 0.54; % Manipulator 링크 길이 [m]

% 원하는 관절 각도 (Desired Joint Angle)
theta = zeros(N,2); % Joint 각도 초기화

for k=1:N,
   theta(k,1) = 0.5*pi + 0.5*sin(pi*t(k));
   theta(k,2) = 0.5*(1-cos(pi*t(k)));
end

%% Forward Kinematics 계산 (Joint coordinate -> Cartesian coordinate)

x = zeros(N,3); y = zeros(N,3);
for k=1:N
%    x(k,1) = 0; 
%    y(k,1) = 0;
%    x(k,2) = L1*cos(theta(k,1));
%    y(k,2) = L1*sin(theta(k,1));
%    x(k,3) = x(k,2) + L2*cos(theta(k,1) + theta(k,2));
%    y(k,3) = y(k,2) + L2*sin(theta(k,1) + theta(k,2));
   
   x(k,:) = [0, L1*cos(theta(k,1)), L1*cos(theta(k,1)) + L2*cos(theta(k,1) + theta(k,2))];
   y(k,:) = [0, L1*sin(theta(k,1)), L1*sin(theta(k,1)) + L2*sin(theta(k,1) + theta(k,2))];
end

%% animation

figure('color','w');

for k=1:5:N
    plot(x(k,1),y(k,1),'ko'); hold on; % Joint 1
    plot(x(k,2),y(k,2),'ko'); hold on; % Joint 2
    plot(x(k,3),y(k,3),'ko'); hold on; % Joint 3
    plot([x(k,1) x(k,2)],[y(k,1) y(k,2)],'b','linewidth',2); hold on;
    plot([x(k,2) x(k,3)],[y(k,2) y(k,3)],'r','linewidth',2); hold on;
    plot(x(1:k,3),y(1:k,3),'c.'); % End point
    axis([-1 1 0 1])
    grid on;
    drawnow;
    hold off;
    
end

