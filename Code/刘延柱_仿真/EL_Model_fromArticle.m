close all;
clc;
i = 1;% 左臂支撑 i = 1，右臂支撑 i = 2
step = 180;% 计算总步数
Omega = 2*pi;% T = 2*pi/Omega = 1s
theta = pi/4;% 章动角
psi = linspace(0,pi,step);
a = 0.50;% 肩线与重心距离，单位 m
b = 0.20;% 肩距的一半，单位 m
m = 64;% 鞍马机器人总重，单位 kg
Ac = 10.7;% 人体中心惯量矩
Bc = 11.0;% 人体中心惯量矩
Cc =  0.6;% 人体中心惯量矩
A = Ac + m*a^2;
B = Bc + m*(a^2+ b^2);
C = Cc + m*b^2;
E = (-1)^i * m*a*b;
g = 9.8;% 重力加速度
Nx = zeros(1,180);
Ny = zeros(1,180);
Nz = zeros(1,180);
Mx = zeros(1,180);
My = zeros(1,180);
Mz = zeros(1,180);
for j = 1:step 
    % 肩关节的支撑力 N 相对于 O-XYZ 的投影式
    Nx(j) = -(-1)^i * 2 * m * Omega^2 * b * (1- cos(theta))*cos(psi(j));
    Ny(j) = m*Omega^2*(-a*sin(theta) + (-1)^i*2*b*(1 - cos(theta))*sin(psi(j)));
    Nz(j) = m*(g + (-1)^i*Omega^2*b*sin(theta)*sin(psi(j)));
    % 肩关节的肌肉控制力矩 M 相对于 O-XYZ 的投影式
    Mx(j) = -Omega^2*(C + (A - C)*cos(theta) + (B - A)*(1 - (2 - cos(theta))*cos(theta)^2))*sin(theta) ...
            -  2* Omega^2*E*cos(theta)*(1 - cos(theta))*sin(psi(j))+ m*g*(a*sin(theta) - (-1)^i*b*cos(theta)*sin(psi(j)));
    My(j) = Omega^2 * ((B - A)*(1 - 2*cos(theta))*sin(theta)*sin(psi(j)) - 2*E*cos(theta)*(1 - cos(theta)))*cos(psi(j)) - (-1)^i*m*g*b*sin(psi(j));
    Mz(j) = Omega^2 * (2 *(A-B) * sin(theta)* sin(psi(j)) + E*(1 - 2*cos(theta))) * sin(theta)* cos(psi(j));
end
figure(1)
plot(psi,Nx);
hold on
plot(psi,Ny);
hold on
plot(psi,Nz);
title('进动角ψ-肩关节支撑力N关系');
legend('Nx','Ny','Nz');
xlabel('ψ/(rad)');
ylabel('N/(N)');
figure(2)
plot(psi,-Mx);
hold on
plot(psi,My);
hold on
plot(psi,Mz);
title('进动角ψ-肩关节肌肉控制力矩M关系');
legend('-Mx','My','Mz');
xlabel('ψ/(rad)');
ylabel('M/(N·m)');