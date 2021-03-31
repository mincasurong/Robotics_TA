clear all; close all; clc;

%% 시뮬레이션 파라미터

T = 0.001; % Sampling period [sec] 
Tfinal = 2; % 시뮬레이션 최종 시간 [sec] 
t = 0:T:Tfinal; % 시뮬레이션 시간 정의 [sec]
N = length(t); % 시뮬레이션 Sample 수

L1 = 0.41; L2 = 0.54; % Manipulator 링크 길이 [m]
r = 0.2; % 원의 반지름

% 원하는 관절 각도 및 위치
P = zeros(N,2); % End point 초기화
P_dot = zeros(N,2); % End Point 속도 초기화
theta = zeros(N,2); % Joint 각도 초기화
% P0 = [0.5 0.2]; % Center

%% End-point 정의
for k=1:N
   th(k) = 2*pi*(k-1)/N; % 0~2pi
   P(k,:) = [0.5 + r*cos(th(k)), 0.2+r*sin(th(k))]; % End point(Circle)
% P(k,1:2)=P0+0.01*[16*(sin(th(k)))^3, 13*cos(th(k))-5*cos(2*th(k))-2*cos(3*th(k))-cos(4*th(k))]; % Heart
end

for k=2:N
    P_dot(k,:) = (P(k,:)-P(k-1,:))/T; % End-point 속도(수치 미분)
end

theta(1,:) = [330, 80]*pi/180; % 각도 초기값 정의

%% Jacobian 계산
J = zeros(N,4);
Jacobian = zeros(2,2);

for k=1:N-1
    J(k,1) = -L1*sin(theta(k,1)) -L2*sin(theta(k,1)+theta(k,2));
    J(k,2) = -L2*sin(theta(k,1)+theta(k,2));
    J(k,3) = L1*cos(theta(k,1)) + L2*cos(theta(k,1)+theta(k,2));
    J(k,4) = L2*cos(theta(k,1)+theta(k,2));
    
    Jacobian = [J(k,1) J(k,2);
                J(k,3) J(k,4)];
            
    Jacobian_inverse = inv(Jacobian);
    
    theta(k+1,:) = theta(k,:) + (Jacobian_inverse * P_dot(k,:)')'*T;
    
end


%% Forward Kinematics 계산 (Joint coordinate -> Cartesian coordinate)

x = zeros(N,3); y = zeros(N,3);
for k=1:N
   x(k,1) = 0; 
   y(k,1) = 0;
   x(k,2) = L1*cos(theta(k,1));
   y(k,2) = L1*sin(theta(k,1));
   x(k,3) = x(k,2) + L2*cos(theta(k,1) + theta(k,2));
   y(k,3) = y(k,2) + L2*sin(theta(k,1) + theta(k,2));
   
%    x(k,:) = [0, L1*cos(theta(k,1)), L1*cos(theta(k,1)) + L2*cos(theta(k,1) + theta(k,2))];
%    y(k,:) = [0, L1*sin(theta(k,1)), L1*sin(theta(k,1)) + L2*sin(theta(k,1) + theta(k,2))];
end

%% Graph

figure('color','w');

for k=1:50:N
    plot(P(:,1),P(:,2),'g','linewidth',4); hold on; % Predefined trajectory
    plot(x(k,1),y(k,1),'ko'); hold on; % Joint 1
    plot(x(k,2),y(k,2),'ko'); hold on; % Joint 2
    plot(x(k,3),y(k,3),'ko'); hold on; % Joint 3
    plot([x(k,1) x(k,2)],[y(k,1) y(k,2)],'b','linewidth',2); hold on; % Link 1
    plot([x(k,2) x(k,3)],[y(k,2) y(k,3)],'r','linewidth',2); hold on; % Link 2
    plot(x(1:100:k,3),y(1:100:k,3),'b','marker','.','markersize',10); % End point
    axis([-0.5 1 -0.5 1])
    grid on;
    drawnow;
    hold off;
    
end

