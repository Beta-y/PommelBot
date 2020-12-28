close all;
clc;
i = 1;% 左臂支撑 i = 1，右臂支撑 i = 2
step = 180;
Omega = 2*pi;% T = 2*pi/Omega = 1s
theta = pi/4;
%phi = ?;
psai = linspace(0,pi,step);
a = 0.50;% m
b = 0.20;% m
m = 64;% kg
Ac = 10.7;
Bc = 11.0;
Cc =  0.6;
A = Ac + m*a^2;
B = Bc + m*(a^2+ b^2);
C = Cc + m*b^2;
E = (-1)^i * m*a*b;
g = 9.8;
Nx = zeros(1,180);
Ny = zeros(1,180);
Nz = zeros(1,180);
Mx = zeros(1,180);
My = zeros(1,180);
Mz = zeros(1,180);
for j = 1:step 
    % 肩关节的支撑力 N 相对于 O-XYZ 的投影式
    Nx(j) = -(-1)^i * 2 * m * Omega^2 * b * (1- cos(theta))*cos(psai(j));
    Ny(j) = m*Omega^2*(-a*sin(theta) + (-1)^i*2*b*(1 - cos(theta))*sin(psai(j)));
    Nz(j) = m*(g + (-1)^i*Omega^2*b*sin(theta)*sin(psai(j)));
    % 肩关节的肌肉控制力矩 M 相对于 O-XYZ 的投影式
    Mx(j) = -Omega^2*(C + (A - C)*cos(theta) + (B - A)*(1 - (2 - cos(theta))*cos(theta)^2))*sin(theta) ...
            -  2* Omega^2*E*cos(theta)*(1 - cos(theta))*sin(psai(j))+ m*g*(a*sin(theta) - (-1)^i*b*cos(theta)*sin(psai(j)));
    My(j) = Omega^2 * ((B - A)*(1 - 2*cos(theta))*sin(theta)*sin(psai(j)) - 2*E*cos(theta)*(1 - cos(theta)))*cos(psai(j)) - (-1)^i*m*g*b*sin(psai(j));
    Mz(j) = Omega^2 * (2 *(A-B) * sin(theta)* sin(psai(j)) + E*(1 - 2*cos(theta))) * sin(theta)* cos(psai(j));
end
figure(1)
plot(psai,Nx);
hold on
plot(psai,Ny);
hold on
plot(psai,Nz);
legend('Nx','Ny','Nz');
figure(2)
plot(psai,Mx);
hold on
plot(psai,My);
hold on
plot(psai,Mz);
legend('Mx','My','Mz');

