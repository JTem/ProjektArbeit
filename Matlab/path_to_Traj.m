fid = fopen('path_sine_snake_transformed_PA_Scheele.csv');
readData = textscan(fid,'%f %f %f %f %f %f %f %f %f %f %f %f', 'Delimiter', ',');
x = readData{1,1}(:,1);
y = readData{1,2}(:,1);
z = readData{1,3}(:,1);

xx = readData{1,4}(:,1);
xy = readData{1,5}(:,1);
xz = readData{1,6}(:,1);

yx = readData{1,7}(:,1);
yy = readData{1,8}(:,1);
yz = readData{1,9}(:,1);

zx = readData{1,10}(:,1);
zy = readData{1,11}(:,1);
zz = readData{1,12}(:,1);

%geschw in mm/s
vel = 20;

for j = 1:length(x)
   
    if j<length(x)-1  
        
        dx = x(j+1)-x(j);
        dy = y(j+1)-y(j);
        dz = z(j+1)-z(j);
        
        dl = sqrt(dx*dx+dy*dy+dz*dz);
        
        dt = dl/vel;
        
        
        R1 =  [xx(j) yx(j) zx(j)
              xy(j) yy(j) zy(j)
              xz(j) yz(j) zz(j)];
        q1 = quaternion(rotm2quat(R1));
         
        
        R2 =  [xx(j+1) yx(j+1) zx(j+1)
              xy(j+1) yy(j+1) zy(j+1)
              xz(j+1) yz(j+1) zz(j+1)];
        q2 = quaternion(rotm2quat(R2));
        
       % dt = 0.1
        q_dot  = (q2-q1)*(1/dt);
        
        w  = 2*q_dot*conj(q1);
        
        [n, ex, ey, ez] = parts(w);
        omega = [ex, ey, ez];
        vel1 =norm([dx/dt,  dy/dt, dz/dt]);
        dq=q1*conj(q2);
        [n1, ex1, ey1, ez1] = parts(dq);
        if n1 < 0 
            n1 = 1;
        end
        theta=2*acos(n1);
        %vell = norm(vel)
        
        angvl = norm(omega);
        
        if angvl > 2
            wmax=2;
            dt=theta/wmax;
        end
        
 
        transM(j,1) = x(j);
        transM(j,2) = y(j);
        transM(j,3) = z(j);
    
        transM(j,4) = xx(j);
        transM(j,5) = xy(j);
        transM(j,6) = xz(j);
    
        transM(j,7) = yx(j);
        transM(j,8) = yy(j);
        transM(j,9) = yz(j);
    
        transM(j,10) = zx(j);
        transM(j,11) = zy(j);
        transM(j,12) = zz(j);
        transM(j,13) = dt;
    end


end
csvwrite('test_traj_angepasst.csv', transM);

