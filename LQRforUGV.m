% copyright @ Ezekiel Mok
% mozhb@mail2.sysu.edu.cn
% 2026/05/12
% clc; clear; close all;

dt = 0.001;
T  = 30;
t  = 0:dt:T;
N  = length(t);

% 车辆参数
m   = 1000;      % 质量
Jz  = 1200;      % 横摆转动惯量
lf  = 1.2;       % 质心至前轴
lr  = 1.6;       % 质心至后轴
Cf  = 80000;     % 前轴侧偏刚度
Cr  = 80000;     % 后轴侧偏刚度
xi  = 0.01;      % 控制点前移量
a   = 0.5;       % 后轴纵向力比例 (0: 前驱; 1: 后驱)

%% =========================
% 无人机参数
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

p.Kp_att = 0.5*[8 8 4];
p.Kd_att = 0.5*[2 2 1];
p.Kp_pos = [3.0, 3.0, 6.0];
p.Kd_pos = [3.0, 3.0, 5.0];

% 积分增益（用于消除微小静差）
p.Ki_pos = [0.2, 0.2, 0.5];  % 可调整

x_uav = zeros(12,1);

% 参考轨迹（模块1格式）
start_ang = 0.55*pi;
end_ang   = 2.25*pi;

% 参数轨迹
x0 = @(s) -40*cos(s + 0.5);
y0 = @(s)  16*sin(2*s + 1);

% 参数映射
s = linspace(start_ang, end_ang, length(t));

% 参考轨迹
x_d = x0(s);
y_d = y0(s);

% 一阶导
dx_d = gradient(x_d, dt);
dy_d = gradient(y_d, dt);

% 二阶导
ddx_d = gradient(dx_d, dt);
ddy_d = gradient(dy_d, dt);

% 初始状态 (起点尽量接近轨迹，减小初始冲击)
X   = 20;
Y   = -12.5;
phi = 0;
vx  = 5;         % 初始速度匹配参考速度 10 m/s
vy  = 0;
r   = 0;
x_state = [X; Y; phi; vx; vy; r];

% 控制器增益 (可适当降低以减小控制量幅值)
Kp = diag([3, 3]);
Kd = diag([5, 5]);

% 日志
log_x   = zeros(6, N);
log_u   = zeros(2, N);   % [Fx; delta_f]
log_e   = zeros(2, N);
log_xd   = zeros(2, N);
Log_x_uav = zeros(12,N);
delta_f = 0;   % 前轮转角初值
Fx = 0;

%% 编队部分
agent0 = agentclass2dim(X, Y, 0,0,0);% 虚拟领航者
a_interg=0;
aa_interg=0;
b_interg=0;
bb_interg=0;
sm_p1=[-2.5,2.5,0]';
sm_p2=[-2.5,-2.5,0]';
sm_p3=[-2.5,2.5,0]';
sigma_p = [sm_p1, sm_p2, sm_p3];
sigma_p1 = [sm_p1, sm_p2, sm_p3];
agent1 = agentclass2dim(X, Y, 0,0,0);% 虚拟领航者
agent2 = agentclass2dim(X, Y, 0,0,0);% 虚拟领航者
agent3 = agentclass2dim(X, Y, 0,0,0);% 虚拟领航者

j=0;
t = zeros(N,1);

% 初始化上一时刻的无人机参考速度（正确值）
sigma_p1_init = sigma_p1;
x_uav(1:3) = [X + sigma_p1_init(1,2);
              Y + sigma_p1_init(2,2);
              0 + sigma_p1_init(3,2)];
x_uav(4:6) = [vx; vy; 0];
ref_UAV_vel_last = [vx; vy; 0];   % 初始时刻无队形速度

U = zeros(N,4);
Epos = zeros(N,3);
Eatt = zeros(N,3);
E_norm = zeros(N,1);

for k = 1:N
    j = (k-1)*dt;
    t(k) = j;
    
    time_start_scaleup   = N*dt/3;
    time_span_scaledown  = 0.5;
    if j>=time_start_scaleup && j<=time_start_scaleup+time_span_scaledown
        a_temp = Accel_slow_fast(time_start_scaleup, time_span_scaledown, j, 0.5);
        a_interg = a_interg + a_temp * dt;
        aa_interg = aa_interg + a_interg * dt;
    end
    
    time_start_scaledown = N*dt/1.25;
    time_span_scaledown  = 1;
    if j>=time_start_scaledown && j<=time_start_scaledown+time_span_scaledown
        a_temp = -Accel_slow_fast(time_start_scaledown, time_span_scaledown, j, 0.5);
        a_interg = a_interg + a_temp * dt;
        aa_interg = aa_interg + a_interg * dt;
    end
    
    % ---------- 缩放结束后强制归零（消除X常差） ----------
    if j > time_start_scaledown + time_span_scaledown
        aa_interg = 0;
        a_interg = 0;
    end
    
    offset = [0,0,0];
    sigma_p1_last = sigma_p1;
    for i = 1:3
        sigma_p1(1:2,i) = Aff('A', phi, 1+aa_interg, 1+aa_interg, offset(i), 0, sigma_p(1:2,i));
    end
    
    % ---------- 队形速度改为零（消除缩放冲击） ----------
    sigma_pv = [0; 0; 0];   % 仅靠车辆速度牵引，不引入缩放速度
    
    agent1.record_x(k) = X + sigma_p1(1,2);
    agent1.record_y(k) = Y + sigma_p1(2,2);
    agent2.record_x(k) = X + sigma_p1(1,1);
    agent2.record_y(k) = Y + sigma_p1(2,1);
    
    X   = x_state(1); Y = x_state(2); phi = x_state(3);
    vx  = x_state(4); vy = x_state(5); r = x_state(6);
    vx = max(vx, 0.1);
    
    % ---- 控制点坐标与速度 ----
    mu   = X + xi*cos(phi);
    nu   = Y + xi*sin(phi);
    
    dmu  = vx*cos(phi) - vy*sin(phi) - xi*r*sin(phi);
    dnu  = vx*sin(phi) + vy*cos(phi) + xi*r*cos(phi);
    
    % ---- 参考信号 ----
    mu_d   = x_d(k);   nu_d   = y_d(k);
    x_d_state = [mu_d - xi*cos(phi); nu_d - xi*sin(phi)];
    dmu_d  = dx_d(k);  dnu_d  = dy_d(k);
    ddmu_d = ddx_d(k); ddnu_d = ddy_d(k);
    
    % ---- 误差 ----
    e  = [mu - mu_d;   nu - nu_d];
    de = [dmu - dmu_d; dnu - dnu_d];
    
    % ---- 当前后轴侧向力 (与控制量无关) ----
    alpha_r = -(vy - lr*r)/vx;
    Fyr = Cr * alpha_r;
    alpha_f = delta_f - (vy + lf*r)/vx;
    Fyf = Cf * alpha_f;
    
    % ---- 构建严格仿射型 H + B*u ----
    H1 = -Fyr*sin(phi)/m + xi*(lf*(Fyf*cos(delta_f) + a*Fx*sin(delta_f)) - lr*Fyr)*sin(phi)/Jz - xi*r^2*cos(phi);
    H2 =  Fyr*cos(phi)/m - xi*(lf*(Fyf*cos(delta_f) + a*Fx*sin(delta_f)) - lr*Fyr)*cos(phi)/Jz - xi*r^2*sin(phi);
    H  = [H1; H2];
    
    % 分别定义三角函数，避免混淆
    c_phi = cos(phi); s_phi = sin(phi);
    c_df  = cos(delta_f); s_df  = sin(delta_f);
    c_dp  = cos(delta_f+phi); s_dp = sin(delta_f+phi);
    
    % B_orig (运动学部分)
    B_orig = (1/m) * [ a*c_phi + (1-a)*c_dp,  -s_dp;
                       a*s_phi + (1-a)*s_dp,   c_dp ];
    
    % ΔB (横摆动力学贡献)
    dB_Fx1  = -xi * s_phi * lf * (1-a) * s_df / Jz;
    dB_Fyf1 = -xi * s_phi * lf * c_df / Jz;
    dB_Fx2  =  xi * c_phi * lf * (1-a) * s_df / Jz;
    dB_Fyf2 =  xi * c_phi * lf * c_df / Jz;
    
    dB = [ dB_Fx1, dB_Fyf1;
           dB_Fx2, dB_Fyf2 ];
    B = B_orig + dB;
    
    % ---- 虚拟控制律 v = ÿ_ref - Kd*ė - Kp*e ----
    v = [ddmu_d; ddnu_d] - Kd*de - Kp*e;
    
    % ---- 反馈线性化求解 ----
    u = pinv(B)* (v - H);
    Fx     = u(1);
    Fyf_des = u(2);
    
    Fx = max(min(Fx, 5000), -5000);
    
    % ---- 由期望侧向力反解前轮转角 ----
    delta_f = Fyf_des / Cf + (vy + lf*r)/vx;
    delta_f = max(min(delta_f, pi/4), -pi/4);
    
    % ---- 更新轮胎力 (用于实际动力学) ----
    alpha_f = delta_f - (vy + lf*r)/vx;
    alpha_r = -(vy - lr*r)/vx;
    Fyf = Cf * alpha_f;
    Fyr = Cr * alpha_r;
    
    % ---- 车辆动力学 ----
    Fx_total = (1-a)*Fx*cos(delta_f) - Fyf*sin(delta_f) + a*Fx;
    Fy_total = (1-a)*Fx*sin(delta_f) + Fyf*cos(delta_f) + Fyr;
    
    dvx = Fx_total/m + vy*r;
    dvy = Fy_total/m - vx*r;
    dr  = (lf*(Fyf*cos(delta_f) + (1-a)*Fx*sin(delta_f)) - lr*Fyr) / Jz;
    
    dX = vx*cos(phi) - vy*sin(phi);
    dY = vx*sin(phi) + vy*cos(phi);
    dphi = r;
    
    dx = [dX; dY; dphi; dvx; dvy; dr];
    x_state = x_state + dt * dx;
    
    % ---------- 计算无人机参考加速度 ----------
    ref_UAV_vel_now = [vx; vy; 0];   % 队形速度为零，只用车速
    if k == 1
        ref_UAV_acc = zeros(3,1);
    else
        ref_UAV_acc = (ref_UAV_vel_now - ref_UAV_vel_last) / dt;
    end
    ref_UAV_vel_last = ref_UAV_vel_now;
    
    % 无人机参考状态（位置、速度、加速度、偏航角）
    ref_UAV1 = [X + sigma_p1(1,2)-5; 
                Y + sigma_p1(2,2); 
                0 + sigma_p1(3,2); 
                vx; 
                vy; 
                0; 
                ref_UAV_acc; 
                phi];
    
    %% ===== 控制器 =====
    u_UAV = controller(x_uav, ref_UAV1, p);
    U(k,:) = u_UAV';

    %% ===== 误差采样（用于后续绘图） =====
    pos_UAV = x_uav(1:3);
    phi_UAV = x_uav(7); theta_UAV = x_uav(8); psi_UAV = x_uav(9);
    pd_UAV = ref_UAV1(1:3);
    psi_UAV_d = ref_UAV1(10);
    ep = pos_UAV - pd_UAV;

    % 期望姿态（用于误差绘图，仅作参考）
    ax_ref = ref_UAV1(7); ay_ref = ref_UAV1(8);
    phi_UAV_d = (1/p.g)*(ax_ref*sin(psi_UAV_d) - ay_ref*cos(psi_UAV_d));
    theta_UAV_d = (1/p.g)*(ax_ref*cos(psi_UAV_d) + ay_ref*sin(psi_UAV_d));
    eatt = [phi_UAV - phi_UAV_d; theta_UAV - theta_UAV_d; psi_UAV - psi_UAV_d];

    Epos(k,:) = ep';
    Eatt(k,:) = eatt';
    E_norm(k) = norm(ep);

    % RK4 更新无人机状态
    k1 = fun_calculation(x_uav, ref_UAV1, p);
    k2 = fun_calculation(x_uav + dt/2*k1, ref_UAV1, p);
    k3 = fun_calculation(x_uav + dt/2*k2, ref_UAV1, p);
    k4 = fun_calculation(x_uav + dt*k3, ref_UAV1, p);
    x_uav = x_uav + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    
    Log_x_uav(:,k) = x_uav;
    
    % ---- 记录 ----
    log_x(:,k) = x_state;
    log_xd(:,k) = x_d_state;
    log_u(:,k) = [Fx; delta_f];
    log_e(:,k) = e;
end

%% 绘图部分（保持原样）
figure
plot(Log_x_uav(1,:), Log_x_uav(2,:), 'LineWidth',1.5);
title('无人机三维轨迹');
hold on 
plot(log_xd(1,:), log_xd(2,:), 'r-', 'LineWidth', 1.5); hold on;
plot(log_x(1,:), log_x(2,:), 'b--', 'LineWidth', 3);
plot(agent1.record_x(:), agent1.record_y(:), 'b--', 'LineWidth', 3);
plot(agent2.record_x(:), agent2.record_y(:), 'g--', 'LineWidth', 3);
legend('UAV', 'Ref', 'Vehicle', 'Agent1', 'Agent2');
xlabel('X'); ylabel('Y');
title('Trajectory Tracking');
grid on; axis equal;

figure;
subplot(4,1,1)
plot(t, log_e(1,:), 'b--', 'LineWidth', 3); hold on;
plot(t, log_e(2,:), 'r--', 'LineWidth', 3);
xlabel('t (s)'); ylabel('norm ');
legend('e_\mu','e_\nu');
title('Formation Tracking Error of UGV');
grid off;

subplot(4,1,2)
plot(t, rad2deg(log_u(2,:)),'b-', 'LineWidth', 3);
xlabel('t (s)'); ylabel('\delta_f (deg)'); title('Steering Angle of UGV');
grid off;

subplot(4,1,3)
plot(t, log_u(1,:), 'b-', 'LineWidth', 3);
xlabel('t (s)'); ylabel('F_x (N)'); title('Longitudinal Force of UGV');
grid off;
subplot(4,1,4)
hold on; box on;

grid off
ylabel( 'Estimating error (m)');
xlabel('t (s)');
plot(sim.t, trace_esterr1, Color=agcolor1,LineStyle='--',LineWidth=3)
plot(sim.t, trace_esterr2, Color=agcolor2,LineStyle='--',LineWidth=3)
plot(sim.t, trace_esterr3, Color=agcolor3,LineStyle='--',LineWidth=3)
 title('Distributed estimation error');
legend('UGV','UAV1','UAV2','Location','northEast');
axis([0,24,0,1])
figure;
plot(t,Epos(:,1),'r','LineWidth',1.5); hold on;
plot(t,Epos(:,2),'g','LineWidth',1.5);
plot(t,Epos(:,3),'b','LineWidth',1.5);
grid on; xlabel('Time (s)'); ylabel('Position Error (m)');
legend('e_x','e_y','e_z'); title('Position Tracking Error');

figure;
plot(t,E_norm,'k','LineWidth',2);
grid on; xlabel('Time (s)'); ylabel('||e_p||');
title('Position Error Norm');

figure;
plot(t,Eatt(:,1),'r','LineWidth',1.5); hold on;
plot(t,Eatt(:,2),'g','LineWidth',1.5);
plot(t,Eatt(:,3),'b','LineWidth',1.5);
grid on; xlabel('Time (s)'); ylabel('Attitude Error (rad)');
legend('e_\phi','e_\theta','e_\psi');
title('Attitude Tracking Error');

figure;
subplot(4,1,1); plot(t,U(:,1),'LineWidth',1.5); grid on; ylabel('F_T'); title('Control Inputs');
subplot(4,1,2); plot(t,U(:,2),'LineWidth',1.5); grid on; ylabel('\tau_\phi');
subplot(4,1,3); plot(t,U(:,3),'LineWidth',1.5); grid on; ylabel('\tau_\theta');
subplot(4,1,4); plot(t,U(:,4),'LineWidth',1.5); grid on; ylabel('\tau_\psi'); xlabel('Time (s)');

%% =================== 辅助函数 ======================
function acceleration = Accel_slow_fast(time_start, time_span, currentTime, distance_change)
    mu = time_start + time_span/2;
    sigma = time_span/8;
    acceleration = -distance_change * 1/(sqrt(2*pi*sigma^2)) * ...
                   exp(-(currentTime-mu)^2/(2*sigma^2)) * ...
                   (-currentTime+mu)/sigma^2;
end

function p2 = Aff(mode, theta, a, b, xm, ym, p)
    if mode == 'S'
        A = [a,0; 0,b]; B = [0;0];
    elseif mode == 'T'
        A = [1,0; 0,1]; B = [xm; ym];
    elseif mode == 'R'
        A = [cos(-theta), sin(-theta); -sin(-theta), cos(-theta)]; B = [0;0];
    elseif mode == 'A'
        A = [a,0; 0,b] * [cos(-theta), sin(-theta); -sin(-theta), cos(-theta)]; B = [xm; ym];
    end
    p2 = A*p + B;
end

function dx = fun_calculation(x, ref, p)
    u = controller(x, ref, p);
    dx = dynamics(x, u, p);
end

%% 无人机控制器（含积分和阻力前馈）
function u = controller(x, ref, p)
    persistent ep_int
    if isempty(ep_int)
        ep_int = zeros(3,1);
    end
    
    pos = x(1:3);
    vel = x(4:6);
    
    phi = x(7); theta = x(8); psi = x(9);
    dphi = x(10); dtheta = x(11); dpsi = x(12);
    
    pd = ref(1:3);
    vd = ref(4:6);
    ad = ref(7:9);
    psi_d = ref(10);
    
    %% 外环 —— 期望加速度（惯性系，含积分和阻力前馈）
    ep = pos - pd;
    ev = vel - vd;
    
    % 积分项更新
    dt = 0.001;  % 与全局dt一致
    ep_int = ep_int + ep * dt;
    % 积分限幅
    ep_int = max(min(ep_int, 2.0), -2.0);
    
    ax = -p.Kp_pos(1)*ep(1) - p.Kd_pos(1)*ev(1) + ad(1) ...
         + (p.Gx/p.M)*vel(1) - p.Ki_pos(1)*ep_int(1);
    ay = -p.Kp_pos(2)*ep(2) - p.Kd_pos(2)*ev(2) + ad(2) ...
         + (p.Gy/p.M)*vel(2) - p.Ki_pos(2)*ep_int(2);
    az = -p.Kp_pos(3)*ep(3) - p.Kd_pos(3)*ev(3) + ad(3) ...
         + (p.Gz/p.M)*vel(3) - p.Ki_pos(3)*ep_int(3);
    
    %% 推力计算
    FT = p.M * (p.g + az);
    FT = max(FT, 0);
    FT = min(FT, 200);
    
    %% 期望推力向量（惯性系）
    t = [ax; ay; az + p.g];
    t_norm = t / norm(t);
    
    %% 姿态解算
    t_rot = [ cos(psi_d)*t_norm(1) + sin(psi_d)*t_norm(2);
              -sin(psi_d)*t_norm(1) + cos(psi_d)*t_norm(2);
               t_norm(3) ];
    
    phi_d   = atan2(-t_rot(2), t_rot(3));
    theta_d = atan2( t_rot(1), sqrt(t_rot(2)^2 + t_rot(3)^2) );
    
    max_angle = deg2rad(45);
    phi_d   = max(min(phi_d,   max_angle), -max_angle);
    theta_d = max(min(theta_d, max_angle), -max_angle);
    
    %% 内环
    e_phi   = phi - phi_d;
    e_theta = theta - theta_d;
    e_psi   = psi - psi_d;
    
    tau_phi   = -p.Kp_att(1)*e_phi   - p.Kd_att(1)*dphi;
    tau_theta = -p.Kp_att(2)*e_theta - p.Kd_att(2)*dtheta;
    tau_psi   = -p.Kp_att(3)*e_psi   - p.Kd_att(3)*dpsi;
    
    max_tau = 5;
    tau_phi   = max(min(tau_phi,   max_tau), -max_tau);
    tau_theta = max(min(tau_theta, max_tau), -max_tau);
    tau_psi   = max(min(tau_psi,   max_tau), -max_tau);
    
    u = [FT; tau_phi; tau_theta; tau_psi];
end

%% 无人机动力学
function dx = dynamics(x, u, p)
    dx1 = x(4); dy1 = x(5); dz1 = x(6);
    phi = x(7); theta = x(8); psi = x(9);
    dphi = x(10); dtheta = x(11); dpsi = x(12);
    
    FT = u(1);
    tau_phi = u(2);
    tau_theta = u(3);
    tau_psi = u(4);
    
    ddx = (FT/p.M)*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) ...
          - (p.Gx/p.M)*dx1;
    ddy = (FT/p.M)*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) ...
          - (p.Gy/p.M)*dy1;
    ddz = (FT/p.M)*(cos(phi)*cos(theta)) - p.g - (p.Gz/p.M)*dz1;
    
    ddphi = (p.L/p.Ix)*tau_phi + dtheta*dpsi*(p.Iy-p.Iz)/p.Ix ...
            - (p.Gphi*p.L/p.Ix)*dphi;
    ddtheta = (p.L/p.Iy)*tau_theta + dphi*dpsi*(p.Iz-p.Ix)/p.Iy ...
              - (p.Gtheta*p.L/p.Iy)*dtheta;
    ddpsi = (p.L/p.Iz)*tau_psi + dphi*dtheta*(p.Ix-p.Iy)/p.Iz ...
            - (p.Gpsi*p.L/p.Iz)*dpsi;
    
    dx = [dx1; dy1; dz1;
          ddx; ddy; ddz;
          dphi; dtheta; dpsi;
          ddphi; ddtheta; ddpsi];
end

%% 估计器函数（保留）
function [esm_x_dot, esm_v_dot] = retrunaikx(estimator, A_matrix, B_matrix, esm_p, esm_pv, x0, v0, agent_id)
    [~, cow] = size(A_matrix);
    sumaijx = zeros(3, 1);
    sumaijv = zeros(3, 1);
    for j = 1:cow
        sumaijx = sumaijx + A_matrix(agent_id, j) .* (esm_p(:, agent_id) - esm_p(:, j));
        sumaijv = sumaijv + A_matrix(agent_id, j) .* (esm_pv(:, agent_id) - esm_pv(:, j));
    end
    e1 = return_sign2(sumaijx + B_matrix(agent_id).*(esm_p(:, agent_id) - x0), estimator.yita1);
    e2 = return_sign2(sumaijv + B_matrix(agent_id).*(esm_pv(:, agent_id) - v0), estimator.yita2);
    esm_x_dot = -estimator.lo1 .* e1 - estimator.lo2 .* sign(sumaijx + B_matrix(agent_id).*(esm_p(:, agent_id) - x0)) + esm_pv(:, agent_id);
    esm_v_dot = -estimator.lo3 .* e2 - estimator.lo4 .* sign(sumaijv + B_matrix(agent_id).*(esm_pv(:, agent_id) - v0));
end

function sign_r1 = return_sign2(e1, r1)
    [scale_e, ~] = size(e1);
    if scale_e == 2
        sign_r1 = [((abs(e1(1)))^r1)*sign(e1(1)), ((abs(e1(2)))^r1)*sign(e1(2))]';
    elseif scale_e == 3
        sign_r1 = [((abs(e1(1)))^r1)*sign(e1(1)), ((abs(e1(2)))^r1)*sign(e1(2)), ((abs(e1(3)))^r1)*sign(e1(3))]';
    else
        sign_r1 = ((abs(e1))^r1)*sign(e1);
    end
end