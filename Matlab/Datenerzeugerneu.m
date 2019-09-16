%% Erzeugen von .csv dateien für unser programm
% hier kann man gewöhnliche parameter objekte erzeugen ( kugel, zylinder,
% torus, etc) und die Kartesischen Punkte dann in einer csv datei
% abspeichern
clear all;
close all;
clf;
%--------------------------------------------------------------------------
% Chose shapes: -vase
%               -zylinder
%               -sphere
%               -skrew
%               -funnel
%               -carambola
%               -diamond
%               -sine
%
% Chose path:   -stair
%               -spiral
%               -meander
%               -snake
%               -hilbert

A = shape('sine', 'snake');

%
%--------------------------------------------------------------------------




figure(1);
ax4 = axes('position', [0.08 0.08 .9 .9]);
%title('Surface'),
xlabel('x'), ylabel('y'), zlabel('z'); hold on;
view(ax4, 42, 24);
daspect([1,1,1]);
plot3(ax4,A(:,1),A(:,2),A(:,3), 'LineWidth',1, 'color', 'b');
function ShapeM = shape(type, pathtype)
showSurface = 0;
increment = 150;
n=5;
intermethod = 'spline';
h  = 0.1;
switch type
    case 'vase'
        su = [0 2*pi];
        sv = [0 20];
    case 'zylinder'
        su = [0 2*pi];
        sv = [0 60];
    case 'sphere'
        su = [-pi pi];
        sv = [0+0.1 pi-0.1];
    case 'skrew'
        increment = 100;
        su = [0 2*pi];
        sv = [0 pi];
    case 'funnel'
        su =[0 2*pi];
        sv = [-20 10];
    case { 'carambola','diamond'}
        su = [-pi pi];
        sv = [.95*-pi/2 .95*pi/2];
        switch type
            case 'carambola'
                T = 2;
                R = 0.5;
                A = 3;
                B = 3;
                C = 3;
            case 'diamond'
                T = 1;
                R = 1;
                A = 3;
                B = 3;
                C = 3;
        end
    case 'sine'
        su = [0 20];
        sv = [0 20];
    otherwise
        error('Chose different Surface - Read Discription')
end

if showSurface
    funx = @(xx,yy) x(xx,yy,type);
    funy = @(xx,yy) y(xx,yy,type);
    funz = @(xx,yy) z(xx,yy,type);
    fs = fsurf(funx, funy, funz ,[su(1) su(2) sv(1) sv(2)]);
    fs.FaceAlpha = 0.5;
    fs.EdgeColor = 'none';
end

switch pathtype
    case 'stair'
        pathMatrix = stair_path(su,sv,increment);
    case 'spiral'
        pathMatrix = spiral_path(su,sv,increment,20*(sv(2)-sv(1))/10000);
    case 'meander'
        pathMatrix = meander_eckig_path(su,sv,increment);
    case 'ecke'
        pathMatrix = ecke(su,sv,increment);
    case 'snake'
        pathMatrix = meander_rund_path(su,sv,increment);
    case 'hilbert'
        pathMatrix = hilbert_path(n,su,sv,increment,intermethod);
    otherwise
        error('Chose different pathtype- Read Discription')
end

for j=1:length(pathMatrix(:,1))
    var_u = pathMatrix(j,1);
    var_v = pathMatrix(j,2);
    
    xp(j) = x(var_u, var_v, type);
    yp(j) = y(var_u, var_v, type);
    zp(j) = z(var_u, var_v, type);
    
    %1. partielle Ableitung nach u (f_u)
    du = [(x(var_u+h,var_v, type)-x(var_u,var_v, type))/h;
        (y(var_u+h,var_v, type)-y(var_u,var_v, type))/h;
        (z(var_u+h,var_v, type)-z(var_u,var_v, type))/h;];
    ndu = norm(du);
    du = du*1/ndu;
    
    %1. partielle Ableitung nach v (f_v)
    dv = [(x(var_u,var_v+h, type)-x(var_u,var_v, type))/h;
        (y(var_u,var_v+h, type)-y(var_u,var_v, type))/h;
        (z(var_u,var_v+h, type)-z(var_u,var_v, type))/h];
    ndv = norm(dv);
    dv = dv*1/ndv;
    
    % Erste Fundamental Form
    E = dot(du,du);
    F = dot(du,dv);
    G = dot(dv,dv);
    
    %Fundamental Form I
    I = [E F
        F G];
    
    %Normalenvektor auf der Fläche
    n = cross(du, dv);
    
    nx(j) = n(1);
    ny(j) = n(2);
    nz(j) = n(3);
    
    dvx(j) = dv(1);
    dvy(j) = dv(2);
    dvz(j) = dv(3);
    
    %2. partielle Ableitung nach u (f_uu)
    ddu = [(x(var_u+2*h,var_v, type)-2*x(var_u+h,var_v, type)+x(var_u,var_v, type))/(h*h);
        (y(var_u+2*h,var_v, type)-2*y(var_u+h,var_v, type)+y(var_u,var_v, type))/(h*h);
        (z(var_u+2*h,var_v, type)-2*z(var_u+h,var_v, type)+z(var_u,var_v, type))/(h*h);];
    
    %2. partielle Ableitung nach v (f_vv)
    ddv = [(x(var_u,var_v+2*h, type)-2*x(var_u,var_v+h, type)+x(var_u,var_v, type))/(h*h);
        (y(var_u,var_v+2*h, type)-2*y(var_u,var_v+h, type)+y(var_u,var_v, type))/(h*h);
        (z(var_u,var_v+2*h, type)-2*z(var_u,var_v+h, type)+z(var_u,var_v, type))/(h*h);];
    
    %2. gemischte partielle Ableitung nach u und v (f_uv)
    dudv = [(x(var_u+h,var_v+h, type)-x(var_u,var_v+h, type)-x(var_u+h,var_v, type)+x(var_u,var_v, type))/(h*h)
        (y(var_u+h,var_v+h, type)-y(var_u,var_v+h, type)-y(var_u+h,var_v, type)+y(var_u,var_v, type))/(h*h)
        (z(var_u+h,var_v+h, type)-z(var_u,var_v+h, type)-z(var_u+h,var_v, type)+z(var_u,var_v, type))/(h*h)];
    
    % zweite Fundamental Form
    L = dot(n, ddu);
    M = dot(n, dudv);
    N = dot(n, ddv);
    
    %Fundamental Form II
    II = [L M
        M N];
    
    %Note: die beiden Krümmungen unterscheiden sich, ich weiß allerdings
    %nicht welche man nun benutzen sollte. hab einfach mal beide
    %aufgeschrieben.
    
    %Berechnung 1 der Oberflächenkrümmung
    Gaussian_curvature = (L*N-M*M)/(E*G-F*F);
    
    %Berechnung 2 der OberfächenKrümmung mit äquivalenter berechnung
    Mean_curvature = 0.5*trace(II/I);
    %Mean_curvature = 0.5*(L*G-2*M*F+N*E)/(E*G-F*F)
    
    gc(j) = Gaussian_curvature;
    mc(j) = Mean_curvature;
end
ShapeM(:,1) = xp;
ShapeM(:,2) = yp;
ShapeM(:,3) = zp;
ShapeM(:,4) = nx;
ShapeM(:,5) = ny;
ShapeM(:,6) = nz;
% ShapeM(:,7) = dvx;
% ShapeM(:,8) = dvy;
% ShapeM(:,9) = dvz;
 ShapeM(:,7) = gc;
 ShapeM(:,8) = mc;
praefix = '_';
suffix = '.csv';
datastring = strcat(type,praefix,pathtype,suffix);
csvwrite(datastring, ShapeM);
    function rf = r(u,v,type)
        
        switch type
            case 'vase'
                rf = 5+7*sin(0.3*v)+0.03*v*v-0.12*v;
            case 'zylinder'
                rf = 30;
            case 'sphere'
                rf=60;
            case 'skrew'
                rf = 2 + sin(7*u + 5*v);
            case 'funnel'
                rf = 25+5*exp(0.25*v);
        end
    end

    function xf = x(u,v, type)
        switch type
            case 'vase'
                xf = r(u,v,type)*cos(u);
            case 'zylinder'
                xf = r(u,v,type)*cos(u);
            case 'sphere'
                xf = r(u,v,type)*cos(u)*sin(v);
            case 'skrew'
                xf = r(u,v,type)*cos(u)*sin(v);
            case 'funnel'
                xf = r(u,v,type)*cos(u);
            case {'carambola','diamond'}
                xf = A*sign(cos(v))*(abs(cos(v))^(2/T))*sign(cos(u))*(abs(cos(u))^(2/R));
            case 'sine'
                xf = u;
        end
    end

    function yf = y(u,v,type)
        switch type
            case 'vase'
                yf = r(u,v,type)*sin(u);
            case 'zylinder'
                yf = r(u,v,type)*sin(u);
            case 'sphere'
                yf = r(u,v,type)*sin(u)*sin(v);
            case 'skrew'
                yf = r(u,v,type).*sin(u)*sin(v);
            case 'funnel'
                yf = r(u,v,type)*sin(u);
            case {'carambola','diamond'}
                yf = B*sign(cos(v))*(abs(cos(v))^(2/T))*sign(sin(u))*(abs(sin(u))^(2/R));
            case'sine'
                yf = v;
        end
    end

    function zf = z(u,v,type)
        
        switch type
            case 'vase'
                zf = 1.5*v;
            case 'zylinder'
                zf = 1.5*v;
            case 'sphere'
                zf = r(u,v,type)+r(u,v,type)*cos(v);
            case 'skrew'
                zf = r(u,v,type)*cos(v);
            case 'funnel'
                zf = v;
            case {'carambola','diamond'}
                zf = C*sign(sin(v))*abs(sin(v))^(2/T);
            case 'sine'
               % zf = 2*sin(0.02*(pi/2*v*u)-pi/90*u*u)*cos(0.7*v);
                zf =0;
        end
    end

%--------------------------------------------------------------------------
% verschiedene Funktionen zur pfaderzeugung
%--------------------------------------------------------------------------
    function M = stair_path(su,sv,increment)
        % su: grenzen in u richtung, bsp: [0 1]
        %
        % sv: grenzen in v richtung, bsp: [-pi pi]
        %
        % increment: anzahl der unterteilungen
        u=su(1);
        v=sv(1);
        i=0;
        inc_u = (su(2)-su(1))/increment;
        inc_v = (sv(2)-sv(1))/20;
        u=su(1)-inc_u;
        v=sv(1);
        while v<=sv(2)
            i=i+1;
            
            if u>su(2)
                v = v + inc_v;
                u=su(1);
            end
            M(i,1) = u;
            M(i,2) = v;
            u = u+inc_u;
        end
    end

    function M = spiral_path(su,sv,increment, steigung)
        %
        % su: grenzen in u richtung, bsp: [0 1]
        %
        % sv: grenzen in v richtung, bsp: [-pi pi]
        %
        % increment: anzahl der unterteilungen
        %
        % steigung: stellt ein wie groß die steigung ist. steigung e (0 1]
        
        i=0;
        inc_u = (su(2)-su(1))/increment;

        u=su(1);
        v=sv(1);
        while v<sv(2)
            i=i+1;

            M(i,1) = u;
            M(i,2) = v;
            u= u+inc_u;
            v = v + steigung;
        end
    end

    function M = meander_eckig_path(su,sv,increment)
        % Meanderkurve mit 90° ecken
        % su: grenzen in u richtung, bsp: [0 1]
        %
        % sv: grenzen in v richtung, bsp: [-pi pi]
        %
        % increment: anzahl der unterteilungen
        u=su(1);
        v=sv(1);
        i=0;
        dir = 1;
        inc_u = (su(2)-su(1))/increment;
        inc_v = (sv(2)-sv(1))/5;
        u=su(1);
        v=sv(1)-inc_v;
        while v<sv(2)
            i=i+1;
            
            if dir
                u= u + inc_u;
            else
                u = u - inc_u;
            end
            
            if u>su(2)+inc_u
                v = v + inc_v;
                u = u - inc_u;
                dir = 0;
            end
            if u<su(1)
                v = v + inc_v;
                u = u + inc_u;
                dir = 1;
            end
            M(i,1) = u;
            M(i,2) = v;
            
        end
    end
    function M = ecke(su,sv,increment)
        % eine eckiger pfad auf einem Objekt
        % su: grenzen in u richtung, bsp: [0 1]
        %
        % sv: grenzen in v richtung, bsp: [-pi pi]
        %
        % increment: anzahl der unterteilungen
        u=su(1);
        v=sv(1);
        i=0;
        dir = 1;
        inc_u = (su(2)-su(1))/increment;
        inc_v = (sv(2)-sv(1))/880;
        u=su(1);
        v=sv(1)-inc_v;
        while v<sv(2)
            i=i+1;
            
            if dir
                u= u + inc_u;
            else
                u = u - inc_u;
            end
            
            if v>sv(2)/2
                dir = 0;
            end
                v = v + inc_v;
     
            M(i,1) = u;
            M(i,2) = v;
            
        end
    end



    function M = meander_rund_path(su,sv,increment)
        % Meander-Kurve mit runden ecken
        % su: grenzen in u richtung, bsp: [0 1]
        %
        % sv: grenzen in v richtung, bsp: [-pi pi]
        %
        % increment: anzahl der unterteilungen
        u=su(1);
        v=sv(1);
        i=0;
        phi = 0;
        dir = 1;
        kreis = 0;
        inc_u = (su(2)-su(1))/increment;
        inc_v = (sv(2)-sv(1))/20;
        u=su(1)+inc_u;
        v=sv(1);
        uf = 0;
        vf = 0;
        while v<sv(2)
            i=i+1;
            
            
            if kreis
                if ~dir
                    u = uf+(inc_v/40)*sind(phi);
                    v = vf+(inc_v/2)-(inc_v/2)*cosd(phi);
                else
                    u = uf-(inc_v/40)*sind(phi);
                    v = vf+(inc_v/2)-(inc_v/2)*cosd(phi);
                end
                phi = phi+5*180/60;
                if phi>183
                    kreis=0;
                    phi=0;
                end
            end
            M(i,1) = u;
            M(i,2) = v;
            if ~kreis
                if dir
                    u = u + inc_u/2;
                else
                    u = u - inc_u/2;
                end
                
                if u>su(2)-inc_u/2
                    kreis = 1;
                    dir = 0;
                end
                if u<su(1)+inc_u
                    kreis = 1;
                    dir = 1;
                end
                vf = v;
                uf = u;
            end
        end
    end

    function M = hilbert_path(n, su, sv, increment, intermethod)
        % eine function die ich im internet gefunden habe ( hilbert ) wird
        % aufgerufen, und zwischen den punkten dieser FKT wird dann interpoliert.
        % es ist einstellbar wie diese interpolation sein soll:
        %
        % n: grad der kurve
        %
        % f: zahlenwert zw 0-1 gibt an, wie fein die unterteilung der punkte sein
        % soll. f=0.5 fügt zwischen zwei punkten einen hinzu f=0.25 fügt 3 hinzu,
        % je kleiner f desto mehr punkte werden hinzugefügt
        %
        % size: größe der Kurve ist veränderbar
        %
        % intermethod: verschiedene arten der Interpolation.
        % wählbar: 'linear', 'cubic', 'spline', -> mehr dazu auf Matlab.com
        f = 25/increment;
        size = 2;
        size_u = su(2)-su(1);
        size_v = sv(2)-sv(1);
        if f>=1
            f = 0.5;
            disp('f muss zwischen Null und eins liegen')
        elseif f<=0
            f = 0.1;
            disp('f muss zwischen Null und eins liegen')
        end
        [a, b] = hilbert(n);
        t = 1:length(a);
        M(:,1) = interp1(t,su(1)+size_u/2+a*size_u,1:f:length(a),intermethod);
        M(:,2) = interp1(t,sv(1)+size_v/2+b*size_v,1:f:length(a),intermethod);
        
        
        %--------------------------------------------------------------------------
        % Subfunktion zur erzeugung der Grund-Kurve
        %--------------------------------------------------------------------------
        
        function [x,y] = hilbert(n)
            %HILBERT Hilbert curve.
            %
            % [x,y]=hilbert(n) gives the vector coordinates of points
            %   in n-th order Hilbert curve of area 1.
            %
            % Example: plot of 5-th order curve
            %
            % [x,y]=hilbert(5);line(x,y)
            %
            %   Copyright (c) by Federico Forte
            %   Date: 2000/10/06
            
            if n<=0
                x=0;
                y=0;
            else
                [xo,yo]=hilbert(n-1);
                x=.5*[-.5+yo -.5+xo .5+xo  .5-yo];
                y=.5*[-.5+xo  .5+yo .5+yo -.5-xo];
                
            end
            
        end
        
    end
end
