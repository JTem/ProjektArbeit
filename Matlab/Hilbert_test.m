%% Klasse zum testen von hilbert curven, 2d und 3d
% Hier zun�chst nur f�r eine 3D kurve. zur erzeugung der 2D kurve muss
% einfach alles mit z weggel�scht werden. ansonsten ist die syntax gleich

clear all;
close all;


% hilbertkurve ist eine funktion und wird aufgerufen und geplottet
% f�r mehr information unten in die function schauen
[x,y,z] = hilbert3_interpolated(2,0.3,80,'spline');
plot3(x,y,z,'Marker','.');


%schleife zum abspeichern der punkte und des tangentenvektors. Die normale
%hab ich einfach mal auf ( 0 0 1 ) gesetzt f�rs erste
for i=1:length(x)
    
    %erste ableitung der kurve
    if i<length(x)
        dx = x(i+1)-x(i);
        dy = y(i+1)-y(i);
        dz = z(i+1)-z(i);
        df = [dx; dy; dz];
    end
    df=df*1/norm(df);
    
    %abspeichern der werte in einer Matrix um diese dann sp�ter in eine csv
    %datei wandeln zu k�nnen
    M(i,1) = x(i);
    M(i,2) = y(i);
    M(i,3) = z(i);
    M(i,4) = 0;
    M(i,5) = 0;
    M(i,6) = 1;
    M(i,7) = df(1);
    M(i,8) = df(2);
    M(i,9) = df(3);
    
end
csvwrite('Hilbert.csv',M);

% Hilbert-2D
function [x,y] = hilbert_interpolated(n, f,size, intermethod)
% eine function die ich im internet gefunden habe ( hilbert ) wird
% aufgerufen, und zwischen den punkten dieser FKT wird dann interpoliert.
% es ist einstellbar wie diese interpolation sein soll:
%
% n: grad der kurve
%
% f: zahlenwert zw 0-1 gibt an, wie fein die unterteilung der punkte sein
% soll. f=0.5 f�gt zwischen zwei punkten einen hinzu f=0.25 f�gt 3 hinzu,
% je kleiner f desto mehr punkte werden hinzugef�gt
%
% size: gr��e der Kurve ist ver�nderbar
%
% intermethod: verschiedene arten der Interpolation.
% w�hlbar: 'linear', 'cubic', 'spline', -> mehr dazu auf Matlab.com
if f>=1
    f = 1;
    disp('f muss zwischen Null und eins liegen')
elseif f<=0
    f = 0.1;
    disp('f muss zwischen Null und eins liegen')
end
[a, b] = hilbert(n);
t = 1:length(a);
x = interp1(t,a*size,1:f:length(a),intermethod);
y = interp1(t,b*size,1:f:length(a),intermethod);


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

% Hilbert-3D
function [x,y,z] = hilbert3_interpolated(n, f, size, intermethod)
% eine function die ich im internet gefunden habe ( hilbert3 ) wird
% aufgerufen, und zwischen den punkten dieser FKT wird dann interpoliert.
% es ist einstellbar wie diese interpolation sein soll:
%
% n: grad der kurve
%
% f: zahlenwert zw 0-1 gibt an, wie fein die unterteilung der punkte sein
% soll. f=0.5 f�gt zwischen zwei punkten einen hinzu f=0.25 f�gt 3 hinzu,
% je kleiner f desto mehr punkte werden hinzugef�gt
%
% size: gr��e der Kurve ist ver�nderbar
%
% intermethod: verschiedene arten der Interpolation.
% w�hlbar: 'linear', 'cubic', 'spline', -> mehr dazu auf Matlab.com
if f>=1
    f = 1;
    disp('f muss zwischen Null und eins liegen')
elseif f<=0
    f = 0.1;
    disp('f muss zwischen Null und eins liegen')
end

[a, b, c] = hilbert3(n);
t = 1:length(a);
x = interp1(t,a*size,1:f:length(a),intermethod);
y = interp1(t,b*size,1:f:length(a),intermethod);
z = interp1(t,c*0.3*size,1:f:length(a),intermethod);

%--------------------------------------------------------------------------
% Subfunktion zur erzeugung der Grund-Kurve
%--------------------------------------------------------------------------
function [x,y,z] = hilbert3(n)
% Hilbert 3D curve.
%
% function [x,y,z] = hilbert3(n) gives the vector coordinates of points
% in n-th order Hilbert curve of area 1.
%
% Example: plot the 3-rd order curve
%
% [x,y,z] = hilbert3(3); plot3(x,y,z)
%   Copyright (c) by Ivan Martynov
%   Inspired by function [x,y] = hilbert(n) made by Federico Forte
%   Date: September 17, 2009
if nargin ~= 1
    n = 2;
end
if n <= 0
    x = 0;
    y = 0;
    z = 0;
else
    [xo,yo,zo] = hilbert3(n-1);
    x = .5*[.5+zo .5+yo -.5+yo -.5-xo -.5-xo -.5-yo .5-yo .5+zo];
    y = .5*[.5+xo .5+zo .5+zo .5+yo -.5+yo -.5-zo -.5-zo -.5-xo];
    z = .5*[.5+yo -.5+xo -.5+xo .5-zo .5-zo -.5+xo -.5+xo .5-yo];
end
end

end



