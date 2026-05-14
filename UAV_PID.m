clc;
clear;
close all;

%% =========================
% 参数
%% =========================
p.M = 5.2;
p.L = 0.25;
p.g = 9.81;

p.Ix = 0.02;
p.Iy = 0.02;
p.Iz = 0.04;

p.Gx = 0.1;
p.Gy = 0.1;
p.Gz = 0.1;

p.Gphi = 0.02;
p.Gtheta = 0.02;
p.Gpsi = 0.02;

p.Kp_pos = 0.5*[3 3 6];
p.Kd_pos = [2 2 4];

p.Kp_att = 5*[8 8 4];
p.Kd_att = [2 2 1];

%% =========================
% 时间设置
%% =========================
dt = 0.01;
T = 20;
N = floor(T/dt);

t = zeros(N,1);

% 状态
X = zeros(N,12);

% 期望轨迹
Xd = zeros(N,3);

% 控制输入
U = zeros(N,4);

% 位置误差
Epos = zeros(N,3);

% 姿态误差
Eatt = zeros(N,3);

% 总误差范数
E_norm = zeros(N,1);

x = zeros(12,1);

%% =========================
% 螺旋轨迹
%% =========================
R = 20;
omega = 0.5;
h = 0.8;

ref = @(tt)[
    R*cos(omega*tt);
    R*sin(omega*tt);
    h*tt;

    -R*omega*sin(omega*tt);
     R*omega*cos(omega*tt);
     h;

    -R*omega^2*cos(omega*tt);
    -R*omega^2*sin(omega*tt);
     0;

     0
];

%% =========================
% RK4循环
%% =========================
for k = 1:N

    tk = (k-1)*dt;
    t(k) = tk;

    X(k,:) = x';

    r = ref(tk);

    % 记录期望轨迹
    Xd(k,:) = r(1:3)';

    %% ===== 控制器 =====
    u = controller(x, r, p);

    % 记录控制输入
    U(k,:) = u';

    %% ===== 误差采样 =====
    pos = x(1:3);

    phi = x(7);
    theta = x(8);
    psi = x(9);

    pd = r(1:3);
    psi_d = r(10);

    % 位置误差
    ep = pos - pd;

    % 期望姿态
    ax = r(7);
    ay = r(8);

    phi_d = (1/p.g)*(ax*sin(psi_d) - ay*cos(psi_d));
    theta_d = (1/p.g)*(ax*cos(psi_d) + ay*sin(psi_d));

    % 姿态误差
    eatt = [
        phi - phi_d;
        theta - theta_d;
        psi - psi_d
    ];

    Epos(k,:) = ep';
    Eatt(k,:) = eatt';

    E_norm(k) = norm(ep);

    %% ===== RK4 =====
    k1 = dynamics(x, u, p);

    k2 = dynamics(x + dt/2*k1, u, p);

    k3 = dynamics(x + dt/2*k2, u, p);

    k4 = dynamics(x + dt*k3, u, p);

    x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);

end

%% =========================
% RMS误差统计
%% =========================
RMS_x = rms(Epos(:,1));
RMS_y = rms(Epos(:,2));
RMS_z = rms(Epos(:,3));

fprintf('\n===== Position RMS Error =====\n');
fprintf('RMS x error = %.4f m\n', RMS_x);
fprintf('RMS y error = %.4f m\n', RMS_y);
fprintf('RMS z error = %.4f m\n', RMS_z);

%% =========================
% 绘图
%% =========================

%% ====== 位置对比 ======
figure(1)

plot(t,X(:,1),'r','LineWidth',1.5);
hold on;

plot(t,X(:,2),'g','LineWidth',1.5);
plot(t,X(:,3),'b','LineWidth',1.5);

plot(t,Xd(:,1),'--r','LineWidth',1.2);
plot(t,Xd(:,2),'--g','LineWidth',1.2);
plot(t,Xd(:,3),'--b','LineWidth',1.2);

grid on;

legend('x','y','z','x_d','y_d','z_d');

xlabel('Time (s)');
ylabel('Position (m)');

title('Position Tracking');

%% ====== 姿态 ======
figure(2)

plot(t,X(:,7),'r','LineWidth',1.5);
hold on;

plot(t,X(:,8),'g','LineWidth',1.5);
plot(t,X(:,9),'b','LineWidth',1.5);

grid on;

legend('\phi','\theta','\psi');

xlabel('Time (s)');
ylabel('Angle (rad)');

title('Attitude');

%% ====== 3D轨迹 ======
figure(3)

plot3(X(:,1),X(:,2),X(:,3),'LineWidth',2);

hold on;

plot3(Xd(:,1),Xd(:,2),Xd(:,3),'--','LineWidth',2);

grid on;

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

legend('Actual','Desired');

title('3D Spiral Tracking');

axis equal;

%% ====== 位置误差 ======
figure(4)
subplot(6,1,1)
plot(t,Epos(:,1),'r--','LineWidth',3);
xlim([-0.1,20.1]);
 ylim([-25,5]);
hold on;

plot(t,Epos(:,2),'g--','LineWidth',3);
plot(t,Epos(:,3),'b--','LineWidth',3);

grid off;

xlabel('Time (s)');
ylabel('Position Error (m)');

legend('e_x','e_y','e_z');

title('Position Tracking Error');

subplot(6,1,2)

plot(t,E_norm,'k','LineWidth',2);
xlim([-0.1,20.1]);
 ylim([-5,25]);
grid off;

xlabel('Time (s)');
ylabel('||e_p||');

title('Position Error Norm');

% %% ====== 姿态误差 ======
% figure(6)
% 
% plot(t,Eatt(:,1),'r','LineWidth',1.5);
% hold on;
% 
% plot(t,Eatt(:,2),'g','LineWidth',1.5);
% plot(t,Eatt(:,3),'b','LineWidth',1.5);
% 
% grid on;
% 
% xlabel('Time (s)');
% ylabel('Attitude Error (rad)');
% 
% legend('e_\phi','e_\theta','e_\psi');
% 
% title('Attitude Tracking Error');
% 
% %% ====== 控制输入 ======
% figure(7)

subplot(6,1,3)
plot(t,U(:,1),'b-','LineWidth',3);
grid off;
xlim([-0.1,20.1]);
 ylim([40,200]);
ylabel('F_T');

title('Control Inputs');

subplot(6,1,4)
plot(t,U(:,2),'b-','LineWidth',3);
grid off;
xlim([-0.1,20.1]);
  ylim([-100,50]);
ylabel('\tau_\phi');

subplot(6,1,5)
plot(t,U(:,3),'b-','LineWidth',3);
xlim([-0.1,20.1]);
grid off;
ylabel('\tau_\theta');

subplot(6,1,6)
plot(t,U(:,4),'b-','LineWidth',3);
xlim([-0.1,20.1]);
grid off;
ylabel('\tau_\psi');
xlabel('Time (s)');

%% =========================
% Controller
%% =========================
function u = controller(x, ref, p)

pos = x(1:3);
vel = x(4:6);

phi = x(7);
theta = x(8);
psi = x(9);

dphi = x(10);
dtheta = x(11);
dpsi = x(12);

pd = ref(1:3);
vd = ref(4:6);
ad = ref(7:9);

psi_d = ref(10);

%% 外环
ep = pos - pd;
ev = vel - vd;

ax = -p.Kp_pos(1)*ep(1) ...
     -p.Kd_pos(1)*ev(1) + ad(1);

ay = -p.Kp_pos(2)*ep(2) ...
     -p.Kd_pos(2)*ev(2) + ad(2);

az = -p.Kp_pos(3)*ep(3) ...
     -p.Kd_pos(3)*ev(3) + ad(3) + p.g;

FT = p.M * sqrt(ax^2 + ay^2 + az^2);

FT = max(FT,0);

%% 姿态期望
phi_d = (1/p.g)*(ax*sin(psi_d) - ay*cos(psi_d));

theta_d = (1/p.g)*(ax*cos(psi_d) + ay*sin(psi_d));

%% 内环
e_phi = phi - phi_d;
e_theta = theta - theta_d;
e_psi = psi - psi_d;

tau_phi = -p.Kp_att(1)*e_phi ...
           -p.Kd_att(1)*dphi;

tau_theta = -p.Kp_att(2)*e_theta ...
             -p.Kd_att(2)*dtheta;

tau_psi = -p.Kp_att(3)*e_psi ...
           -p.Kd_att(3)*dpsi;

u = [
    FT;
    tau_phi;
    tau_theta;
    tau_psi
];

end

%% =========================
% Dynamics
%% =========================
function dx = dynamics(x,u,p)

dx1 = x(4);
dy1 = x(5);
dz1 = x(6);

phi = x(7);
theta = x(8);
psi = x(9);

dphi = x(10);
dtheta = x(11);
dpsi = x(12);

FT = u(1);

tau_phi = u(2);
tau_theta = u(3);
tau_psi = u(4);

%% 平动
ddx = (FT/p.M)*(cos(phi)*sin(theta)*cos(psi) ...
      + sin(phi)*sin(psi)) ...
      - (p.Gx/p.M)*dx1;

ddy = (FT/p.M)*(cos(phi)*sin(theta)*sin(psi) ...
      - sin(phi)*cos(psi)) ...
      - (p.Gy/p.M)*dy1;

ddz = (FT/p.M)*(cos(phi)*cos(theta)) ...
      - p.g ...
      - (p.Gz/p.M)*dz1;

%% 转动
ddphi = (p.L/p.Ix)*tau_phi ...
        + dtheta*dpsi*(p.Iy-p.Iz)/p.Ix ...
        - (p.Gphi*p.L/p.Ix)*dphi;

ddtheta = (p.L/p.Iy)*tau_theta ...
          + dphi*dpsi*(p.Iz-p.Ix)/p.Iy ...
          - (p.Gtheta*p.L/p.Iy)*dtheta;

ddpsi = (p.L/p.Iz)*tau_psi ...
        + dphi*dtheta*(p.Ix-p.Iy)/p.Iz ...
        - (p.Gpsi*p.L/p.Iz)*dpsi;

dx = [
    dx1;
    dy1;
    dz1;

    ddx;
    ddy;
    ddz;

    dphi;
    dtheta;
    dpsi;

    ddphi;
    ddtheta;
    ddpsi
];

end