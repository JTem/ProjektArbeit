%% Erzeugen von einem Turbinenschaufel-pfad
clear all;
close all;
clf;


t = 0;
tt=0;
i = 0;
h=0.1;
figure(1)
ax1 = axes('position',[.1 .1 .8 .8]);hold on;
camproj perspective;
daspect([1 1 1]);

rn = 3;
funx = @(u,v) rn.*cos(u);
funz = @(u,v) rn.*sin(u);
funy = @(u,v) v;
zyl =  fsurf(ax1,funx,funy,funz,[0 pi -10 10]);
zyl.EdgeColor =  'none';
zyl.FaceAlpha = 0.5;
camlight

while tt<=3
    
    i = i+1;
    xe = (1)/(1+1*tt)*0.2*pi/2*cos(t)+pi/2;
    ye = (1)/(1+0.2*tt)*pi*sin(t);
    phi = 10;
    Re = [cosd(phi) -sind(phi)
          sind(phi)  cosd(phi)];
    xq = [xe; ye];
    xw = Re*xq;
    x_path(i) = r(tt)*x(xw(1));
    y_path(i) = y(xw(2));
    z_path(i) = r(tt)*z(xw(1));
    
    
    du = [(r(tt)*x(xw(1)+h)-r(tt)*x(xw(1)))/h
           0
          (r(tt)*z(xw(1)+h)-r(tt)*z(xw(1)))/h];
      
    dv = [0
          (y(xw(2)+h)-y(xw(2)))/h
          0];
    du = du/norm(du);
    dv = dv/norm(dv);
    normal = cross(dv,du);
    tt = tt+0.004;
    t=t+0.1;
    datM(i,1) = x_path(i);
    datM(i,2) = y_path(i);
    datM(i,3) = z_path(i);
    datM(i,4) = normal(1);
    datM(i,5) = normal(2);
    datM(i,6) = normal(3);
end
csvwrite('turbinenschaufel.csv',datM);
plot3(x_path,y_path,z_path,'Marker','.');

function rf = r(t)
rf = 3+t;
end

function xf = x(t)
xf = cos(t);
end

function yf = y(t)
yf = t;
end

function zf = z(t)
zf = sin(t); 
end