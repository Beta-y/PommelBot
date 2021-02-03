close all;
clc;
% i = 1;% 左臂支撑 i = 1，右臂支撑 i = 2
Omega = 2*pi;% T = 2*pi/Omega = 1s
theta = pi/4;% 章动角
d_phi = -Omega;% 自转角速度
d_psi = Omega/cos(theta);
w0 = Omega;
a = 0.50;% 肩线与重心距离，单位 m
b = 0.20;% 肩距的一半，单位 m
m = 64;% 鞍马机器人总重，单位 kg
A0 = 10.7;% 人体中心赤道惯量矩
g = 9.8;% 重力加速度
Msx = zeros(1,181);
Msy = zeros(1,181);
phi = zeros(1,181);
psi = zeros(1,181);
j = 1;
for t = 0:1/180:1.0
    phi(j) = -w0*cos(theta)*t;
    psi(j) = w0*t;
    % 肩关节的肌肉控制力矩 M 相对于 O-XYZ 的投影式
    Msx(j) = m*g*(b*cos(theta)+a*sin(theta))-w0^2*sin(theta)*((A0+m*a^2)*cos(theta)+m*a*b*sin(theta))*cos(phi(j));
    Msy(j) = -w0^2*sin(theta)*((A0+m*a^2)*cos(theta)+m*a*b*sin(theta))*sin(phi(j));
    j = j + 1;
end
t = linspace(0,1.0,181);
figure(1)
plot(t,Msx);
hold on
plot(t,Msy);
title('时间-肩关节肌肉控制力矩M关系');
legend('Msx','Msy');
xlabel('t/(s)');
ylabel('M/(N·m)');
figure(2)
plot(phi,Msx);
hold on
plot(phi,Msy);
title('自转角\phi-肩关节肌肉控制力矩M关系');
legend('Msx','Msy');
xlabel('\phi/(rad)');
ylabel('M/(N·m)');
figure(3)
plot(psi,Msx);
hold on
plot(psi,Msy);
title('进动角\psi-肩关节肌肉控制力矩M关系');
legend('Msx','Msy');
xlabel('\psi/(rad)');
ylabel('M/(N·m)');
figure(4)
plot(psi,phi);
hold on
plot(psi,phi);
title('进动角\psi-自转角\phi关系');
legend('\psi','\phi');
xlabel('\psi/(rad)');
ylabel('\phi/(rad)');