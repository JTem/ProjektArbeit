%% 3D Plotten von .csv Dateien.
% Hier mit Projektion von punkten auf eine ebene

clear all;
close all;
clf;

% Initialisierung von Variablen
showKS = 1;
ddruck = 1;
normalVec = [0;0;1];

% orientierung des Tools einstellen
phi = 0;    % drehung um z
theta = 0;  % drehung um y
psi = 0;    % drehung um x


% Ort und Orientierung der Nozzle
T_Nozzle = [1 0 0 500
    0 1 0 100
    0 0 1 500
    0 0 0 1];

%Datei einlesen
fid = fopen('zylinder_spiral.csv');

tline = fgetl(fid);
element_num = length(find(tline==','))+1;

if element_num == 6
    readData = textscan(fid,'%f %f %f %f %f %f', 'Delimiter', ',');
elseif element_num == 8
    readData = textscan(fid, '%f %f %f %f %f %f %f %f', 'Delimiter',',');
    gCurvature = readData{1,7}(:,1);
    mCurvature = readData{1,8}(:,1);
else
    error('not enough information provided!!!');
end

% Daten einlesen
x = 1*readData{1,1}(:,1);
y = 1*readData{1,2}(:,1);
z = 1*readData{1,3}(:,1);
nx = readData{1,4}(:,1);
ny = readData{1,5}(:,1);
nz = readData{1,6}(:,1);


% größe des objektes ermitteln um graphen und vektoren anpassen zu können
x_min = min(x);
x_max = max(x);
y_min = min(y);
y_max = max(y);
z_min = min(z);
z_max = max(z);
avg_size = ((x_max-x_min)+(y_max-y_min)+(z_max-z_min))/3;
v_size = 0.15*avg_size;


figure(1);
set(gcf, 'position',[10,100,1600,900]);

%Graph 1 für den Pfad
ax1 = axes('position',[0.03 0.04 0.4 1]);
title('Parameter path'), xlabel('x'), ylabel('y'), zlabel('z'); hold on;
set(ax1, 'XLim', [x_min-2*v_size x_max+2*v_size], 'YLim', [y_min-2*v_size y_max+2*v_size], 'ZLim', [z_min-2*v_size z_max+2*v_size]);
view(ax1, 43, 24);
camproj perspective;
daspect([1 1 1]);

%Graph 2 für die Orientierung
ax2 = axes('position',[0.55 0.73 0.4 0.25]);
title('Eulerwinkel der Orientierung'), xlabel('t in %'), ylabel('eulerangles in °'); hold on;
set(ax2, 'XLim', [0 100], 'YLim', [-180 180]);

%Graph 3 für die Krümmung
ax3 = axes('position', [0.55 0.4 0.4 0.25]);
title('Curvature'), xlabel('t in %'), ylabel('curvature');hold on;
set(ax3, 'XLim', [0 100]);

% Graph 4
ax4 = axes('position', [0.55 0.07 0.4 0.25]);
title('Verhältnis Krümmung/Proj_Krümmung'), xlabel('x'), ylabel('y'), zlabel('z'); hold on;
set(ax4, 'XLim', [0 100], 'YLim', [-0.5 1.5]);

% Rotationsmatrix um die Toolorientierung einzustellen
eul = [deg2rad(phi) deg2rad(theta) deg2rad(psi)];
R = eul2rotm(eul, 'ZYX')

% schleife zur vorberechnung. hier werden ableitungen, Krümmung,
% rotationsmatritzen und die dazu gehörigen winkel berechnet. diese werden
% dann in cells/Vektoren abgespeichert. Dies ist effizienter als diese
% jedes mal in der Animationsschleife neu zu berechnen.
for j = 1:length(x)
    
    derf = df(x,y,z,j);
    der2f = ddf(x,y,z,j);
    curv = norm(cross(derf, der2f))/norm(derf)^3;
    curvature_Path(j) = curv;
    
    n = [nx(j); ny(j); nz(j)];
    tan = derf*1/norm(derf);
    bin = cross(n, tan);
    bin = [0;0;1]
    tan = cross(bin, n);
    dPhi(j) = rad2deg(acos(dot(n,normalVec)));
    
%     ks = [tan(1) bin(1) n(1)
%         tan(2) bin(2) n(2)
%         tan(3) bin(3) n(3)];
%     
      ks = [1 0 0
           0 1 0
           0 0 1];
    
    kt = ks*R
    
    KS(j) = {ks};
    KT(j) = {kt};
    
    alpha = rotm2eul(kt, 'ZYX');
    
    aPhi(j) = rad2deg(alpha(1));
    aTheta(j) = rad2deg(alpha(2));
    aPsi(j) = rad2deg(alpha(3));
    
    % Rücktransformation von Nozzle zum Roboterflansch
    
    % Transformatrionsmatrix des Pfades
    transformation_Matrix = [kt(1,1) kt(1,2) kt(1,3) x(j)
        kt(2,1) kt(2,2) kt(2,3) y(j)
        kt(3,1) kt(3,2) kt(3,3) z(j)
        0 0 0 1];
    
    % ti = T_inv(transformation_Matrix);
    % T_flange = T_Nozzle*ti;
    
    % abspeichern der Transformationsmatrix (Pfad)
    transM(j,1) = transformation_Matrix(1,4);
    transM(j,2) = transformation_Matrix(2,4);
    transM(j,3) = transformation_Matrix(3,4);
    
    transM(j,4) = transformation_Matrix(1,1);
    transM(j,5) = transformation_Matrix(2,1);
    transM(j,6) = transformation_Matrix(3,1);
    
    transM(j,7) = transformation_Matrix(1,2);
    transM(j,8) = transformation_Matrix(2,2);
    transM(j,9) = transformation_Matrix(3,2);
    
    transM(j,10) = transformation_Matrix(1,3);
    transM(j,11) = transformation_Matrix(2,3);
    transM(j,12) = transformation_Matrix(3,3);
end
csvwrite('schnecke_xi1.csv', transM);

% in dieser Schleife wird die projizierte Krümmung berechnet.
% die krümmung wird aus 3 punkten berechnet. punkt0 spannt die ebene auf
% und ist der zentrale punkt. punkt1 ist der punkt vor punkt0 und wird auf
% die aufgespannte ebene projiziert. punkt2 ist der punkt nach punkt0 und
% wird auch auf die ebene projiziert
for j = 1:length(x)
    % Vorher abgespeicherte Toolorientierung wird wieder geöffnet
    % diese Punkte sind für das aufspannen der ebene
    kT0 = cell2mat(KT(1,j));
    n0 = kT0(:,3);
    V0 = [x(j); y(j); z(j)];
    
    % folgende punkte sind für den punkt for dem betrachteten punkt
    if j<length(x)
        kT1 = cell2mat(KT(1,j+1));
        n1 =kT1(:,3);
        P01 = [x(j+1); y(j+1); z(j+1)];
        P11 = [x(j+1)+n1(1); y(j+1)+n1(2); z(j+1)+n1(3)];
        [I1, check1] = plane_line_intersect(n0,V0, P01, P11);
    end
    % folgende punkte sind für den punkt nach dem  betrachteten Punkt
    if j>=2
        kT2 = cell2mat(KT(1,j-1));
        n2 = kT2(:,3);
        P02 = [x(j-1); y(j-1); z(j-1)];
        P12 = [x(j-1)+n2(1); y(j-1)+n2(2); z(j-1)+n2(3)];
        [I2, check2] = plane_line_intersect(n0,V0, P02, P12);
        
        % hier wird die krümmung aus den 3 punkten berechnet
        der_proj = I1-V0
        der2_proj = I1-2*V0+I2
        if isnan(der2_proj)
        curvature_Proj(j) = 0
        xi(j) = 0/(curvature_Path(j));
        else
        curv_proj = norm(cross(der_proj, der2_proj))/norm(der_proj)^3;
        curvature_Proj(j) = curv_proj;
        xi(j) = curv_proj/(curvature_Path(j));
        end

    end
end

path = animatedline(ax1,"LineWidth", 1, 'color' , 'b');
plot_curvature = animatedline(ax3,"LineWidth", 2, 'color', 'k');
plot_curv_proj = animatedline(ax3,"LineWidth", 1, 'color', 'm');
plot_alpha = animatedline(ax2,"LineWidth", 1, 'color', 'b');
plot_beta = animatedline(ax2,"LineWidth", 1, 'color', 'g');
plot_gamma = animatedline(ax2,"LineWidth", 1, 'color', 'r');
plot_xi = animatedline(ax4, "lineWidth", 1, 'color', 'b');

% Animationsschleife
for j = 1:length(x)
    
    %plot von 0-100% durchlaufen
    x_plot = 100*(j/length(x));
    addpoints(path, x(j), y(j), z(j));
    kS = cell2mat(KS(1,j));
    kT = cell2mat(KT(1,j));
    
    daPsi = diff(aPsi);
    daTheta = diff(aTheta);
    daPhi = diff(aPhi);
    
    addpoints(plot_gamma, x_plot, aPsi(j));
    addpoints(plot_beta, x_plot, aTheta(j));
    addpoints(plot_alpha, x_plot, aPhi(j));
    addpoints(plot_curvature, x_plot, curvature_Path(j));
    addpoints(plot_curv_proj, x_plot, curvature_Proj(j));
    addpoints(plot_xi, x_plot, xi(j));
    legend(ax2, 'phi', 'theta', 'psi');
    if showKS
        %Koordinatensystem plotten
        qt = quiver3(ax1, x(j), y(j), z(j), kS(1,1), kS(2,1), kS(3,1), 'color', 'r');
        qb = quiver3(ax1, x(j), y(j), z(j), kS(1,2), kS(2,2), kS(3,2), 'color', 'g');
        qn = quiver3(ax1, x(j), y(j), z(j), kS(1,3), kS(2,3), kS(3,3), 'color', 'b');
        set(qt,'AutoScale','on', 'AutoScaleFactor', v_size);
        set(qb,'AutoScale','on', 'AutoScaleFactor', v_size);
        set(qn,'AutoScale','on', 'AutoScaleFactor', v_size);
    end
    
    %Tool plotten (auch als Vektor)
    qTool = quiver3(ax1,x(j), y(j), z(j), kT(1,3), kT(2,3), kT(3,3), 'color', 'k');
    set(qTool,'AutoScale','on', 'AutoScaleFactor', 1.5*v_size,'Linewidth', 2);
    
    %weitere animations syntax
    drawnow limitrate;
    %pause(2);
    
    %koordinatensystem, tool, und head wieder löschen am ende der schleife
    if showKS
        delete(qn);
        delete(qt);
        delete(qb);
    end
    delete(qTool);
    
end

function [I,check]=plane_line_intersect(n,V0,P0,P1)
%plane_line_intersect computes the intersection of a plane and a segment(or
%a straight line)
% Inputs:
%       n: normal vector of the Plane
%       V0: any point that belongs to the Plane
%       P0: end point 1 of the segment P0P1
%       P1:  end point 2 of the segment P0P1
%
%Outputs:
%      I    is the point of interection
%     Check is an indicator:
%      0 => disjoint (no intersection)
%      1 => the plane intersects P0P1 in the unique point I
%      2 => the segment lies in the plane
%      3=>the intersection lies outside the segment P0P1
%
% Example:
% Determine the intersection of following the plane x+y+z+3=0 with the segment P0P1:
% The plane is represented by the normal vector n=[1 1 1]
% and an arbitrary point that lies on the plane, ex: V0=[1 1 -5]
% The segment is represented by the following two points
% P0=[-5 1 -1]
%P1=[1 2 3]
% [I,check]=plane_line_intersect([1 1 1],[1 1 -5],[-5 1 -1],[1 2 3]);
%This function is written by :
%                             Nassim Khaled
%                             Wayne State University
%                             Research Assistant and Phd candidate
%If you have any comments or face any problems, please feel free to leave
%your comments and i will try to reply to you as fast as possible.
I=[0 0 0];
u = P1-P0;
w = P0 - V0;
D = dot(n,u);
N = -dot(n,w);
check=0;
if abs(D) < 10^-7        % The segment is parallel to plane
    if N == 0           % The segment lies in plane
        check=2;
        return
    else
        check=0;       %no intersection
        return
    end
end
%compute the intersection parameter
sI = N / D;
I = P0+ sI.*u;
if (sI < 0 || sI > 1)
    check= 3;          %The intersection point  lies outside the segment, so there is no intersection
else
    check=1;
end
end

function ti =  T_inv(T)
% funktion zum invertieren einer  Transformationsmatrix

r = tform2rotm(T);
p = tform2trvec(T);
p = transpose(p);
rT = transpose(r);
p2 = -rT*p;

ti = [rT(1,1) rT(1,2) rT(1,3) p2(1)
    rT(2,1) rT(2,2) rT(2,3) p2(2)
    rT(3,1) rT(3,2) rT(3,3) p2(3)
    0 0 0 1];
end

function d = df(sx, sy, sz, i)
% erste ableitung einer funktion aus vektoren an der stelle i
if i < length(sx)
    d = [sx(i+1)-sx(i)
        sy(i+1)-sy(i)
        sz(i+1)-sz(i)];
else
    d = [0;0;0];
end
end

function d = ddf(sx, sy, sz, i)
% zweite ableitung einer funktion aus vektoren an der stelle i
if i < length(sx)-1
    d = [sx(i+2)-2*sx(i+1)+sx(i)
        sy(i+2)-2*sy(i+1)+sy(i)
        sz(i+2)-2*sz(i+1)+sz(i)];
else
    d=[0;0;0];
end
end



