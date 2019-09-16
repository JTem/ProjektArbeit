clear all;
close all;
clf;
fid = fopen('angle_schnecke_xi1.csv');

readData = textscan(fid,'%f %f %f %f %f %f %f', 'Delimiter', ',');


q1 = readData{1,1}(:,1);
q2 = readData{1,2}(:,1);
q3 = readData{1,3}(:,1);
q4 = readData{1,4}(:,1);
q4_ = q4(2:2:end);
q5 = readData{1,5}(:,1);
q5_ = q5(2:2:end);
q6 = readData{1,6}(:,1);
q6_ = q6(1:2:end);

% 
 ax1 = axes();
% xlabel('x'), ylabel('y'), zlabel('z'); hold on;
 %view(ax1, 70, 24);
 %camproj perspective;
 %daspect([1 1 1]);
% 
% 
 plot(ax1,q4, '.-','color', 'r'); hold on;
 plot(ax1,q5,'.-', 'color', 'g');
 plot(ax1, q6,'.-','color', 'b');
 legend(ax1, 'q_4', 'q_5', 'q_6');
 saveas(gcf,'sxi1','epsc');

% t=0:pi/50:5*pi;   % to have one complete round
% r = 2;            % radius
% h = 2;            % height
% x = r * sin(t);
% y = r * cos(t);
% z = h/(2*pi) * t;   
% 
% x1 = r*sin(3*pi);
% x2 = r*sin(3.001*pi);
% 
% y1 = r*cos(3*pi);
% y2 = r*cos(3.001*pi);
% 
% z1 = h/(2*pi) * 3*pi; 
% z2 = h/(2*pi) * 3.001*pi; 
% 
% dx = (x2-x1)/(0.001*pi);
% dy = (y2-y1)/(0.001*pi);
% dz = (z2-z1)/(0.001*pi);
% 
% l = sqrt(dx*dx+dy*dy+dz*dz);
% dx = dx/l;
% dy = dy/l;
% dz = dz/l;
% xp = r*sin(3*pi);
% yp = r*cos(3*pi);
% zp = h*1.5;
% 
% v1 = dx*0-dz*dx;
% v2 = dz*dy-dx*0;
% v3 = dx*dx-dy*dy;
% 
% qt = quiver3(ax1,xp,yp,zp, dx, dy, dz,'color', 'r', 'LineWidth',  2);
% qn = quiver3(ax1,xp,yp,zp, -dy, -dx, 0,'color', 'b', 'LineWidth', 2);
% qb = quiver3(ax1,xp,yp,zp, v1, v2, v3,'color', 'g', 'LineWidth', 2);
% plot3(ax1,x,y,z, 'color', 'b');
% %saveas(gcf,'Parameter Kurve','epsc');
% 
