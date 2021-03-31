% dynamic simulation

clear all; clc; close all;

T=0.001;      % Sampling Period [sec]
Tfinal=10;     % 시뮬레이션 시간 [sec]
t=0:T:Tfinal; % 진행 시간 [sec]
N=length(t);  % 시뮬레이션 Sample 갯수 [Samples]

L1=0.41; L2=0.54; % Manipulator 링크 길이 [m]
m1=5; m2=3;       % Mass [kg]
g=9.81;           % 중력가속도 [m/s^2]

%% Generalized coordinate
% th: 각도[rad], dth:각속도[rad/s], ddth:각가속도[rad/s^2]
th=zeros(N,2); dth=zeros(N,2); ddth=zeros(N,2);
tau1=zeros(N,1); tau2=zeros(N,1); tau=zeros(N,2);
th(1,1)=-pi/4;  th(1,2)=0; %초기값 설정

for k=1:N
%     토크 입력(주어진 함수)
%     if k<2000,
        tau1(k)=3*sin(pi*t(k));
        tau2(k)=1*cos(pi*t(k));
        tau(k,:)=[tau1(k) tau2(k)];
%     end
end

%% Simulation
for k=1:N-1;
    
    %관성력항 계산
    M11(k)=1/3*m1*L1^2 + m2*(L1^2+1/3*L2^2 + L1*L2*cos(th(k,2)));
    M12(k)=m2*(1/3*L2^2+1/2*L1*L1*cos(th(k,2)));
    M21(k)=M12(k);
    M22(k)=m2*1/3*L2^2;
    M=[M11(k) M12(k); M21(k) M22(k)];
    M_inv =inv(M);
    
    %코리올리 및 원심력항 계산
    C1(k)=-1/2*m2*L1*L2*sin(th(k,2))*dth(k,2)^2-m2*L1*L2*sin(th(k,2))*dth(k,1)*dth(k,2);
    C2(k)=1/2*m2*L1*L2*sin(th(k,2))*dth(k,1)^2;
    C=[C1(k) C2(k)];
    
    %중력항 계산
    G1(k) = m1*g*1/2*L1*cos(th(k,1))+m2*g*(L1*cos(th(k,1))+1/2*L2*cos(th(k,1)+th(k,2)));
    G2(k) = m2*g*1/2*L2*cos(th(k,1)+th(k,2));
    G=[G1(k) G2(k)];
    
    %forward dynamics (토크입력 -> 각가속도 출력)
    C_input=tau(k,:)-C-G; %+(-2*sign(dth(k,:))-1*dth(k,:));
    ddth(k,:)=(M_inv*C_input');       %다음 샘플의 각가속도 계산
    dth(k+1,:)=dth(k,:)+ddth(k,:)*T;  %다음샘플의 각속도 계산
    th(k+1,:)=th(k,:)+dth(k+1,:)*T;   %다음샘플의 각도 계산
end

x=zeros(N,3); y=zeros(N,3); % 좌표계 위치 초기화
for k=1:N
    x(k,1)=0;
    y(k,1)=0;
    x(k,2)=L1*cos(th(k,1));
    y(k,2)=L1*sin(th(k,1));
    x(k,3)=x(k,2)+L2*cos(th(k,1)+th(k,2));
    y(k,3)=y(k,2)+L2*sin(th(k,1)+th(k,2));
end

figure('color','w');
for k=1:100:N
    plot(x(k,1),y(k,1),'ko'); hold on; % Joint 1
    plot(x(k,2),y(k,2),'bo'); hold on; % Joint 2
    plot(x(k,3),y(k,3),'rs'); hold on; % Joint 3
    plot([x(k,1) x(k,2)],[y(k,1) y(k,2)],'b','linewidth',2); hold on; % Link 1
    plot([x(k,2) x(k,3)],[y(k,2) y(k,3)],'r','linewidth',2); hold on; % Link 2
    plot(x(1:k,3),y(1:k,3),'c.');            % End Point
    axis([-1 1 -1 0])                         % Axis X, Y
    grid on;
    drawnow;
    hold off;
end


figure('color','w');

subplot(211); % Graphs of end point
plot(t,tau1,'b','linewidth',2); hold on;
plot(t,tau2,'r','linewidth',2); hold on;
legend('τ_1','τ_2')
ylabel('Torque(Nm)'); xlabel('time(sec)')

subplot(212); % Graphs of joint angle
plot(t,th(:,1)*180/pi,'b','linewidth',2); hold on;
plot(t,th(:,2)*180/pi,'r','linewidth',2); hold on;
legend('θ_1','θ_2')
ylabel('Angle(deg)'); xlabel('time(sec)')


figure('color','w');

subplot(311); % Graphs of end point
plot(t,x(:,3),'b','linewidth',2); hold on;
ylabel('x3(m)'); xlabel('time(sec)')

subplot(312); % Graphs of joint angle
plot(t,y(:,3),'r','linewidth',2); hold on;
ylabel('y3(m)'); xlabel('time(sec)')

subplot(313); % Graphs of joint angle
plot(x(:,3),y(:,3),'b','linewidth',2); hold on;
ylabel('y3(m)'); xlabel('x3(m)')



