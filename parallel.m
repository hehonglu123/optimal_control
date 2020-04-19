clc;
clear;
close all;
l = 26; %wheelbase - mm
w = 15; %tread - mm
u = 0;
v = 5;  %vehicle speed - m/s
dt =0.8; %Sampling interval - second

x=20; %The strating X coordinate of the vehicle
y=45; %The strating Y coordinate of the vehicle
theta=0; %Automobile body initial inclination
phi=0; %max 45 degree

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Go stright until find the parking place%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_seq=zeros(1,1);
y_seq=zeros(1,1);
theta_seq=zeros(1,1);
phi_seq=zeros(1,1);
figure; %draw the empty picture 
pause(1);
for i = 1:28
    pause(0.3);
    [x,y,theta,v]=state_update(x,y,theta,v,dt,phi,u);

    x_seq(i) = x; %存储 x 坐标到 x 序列store x coordinate to x sequence
    y_seq(i) = y; %store y coordinate to y sequence
    theta_seq(i) = theta; %store theta coordinate to theta sequence
    phi_seq(i) = phi; %store phi coordinate to phi theta sequence
      %fprintf('x = %f, y = %f, theta = %f, phi = %f, dx = %f, dy = %f\n', x, y,theta, phi, delta_x, delta_y);
    x0 = x + w/2*sin(theta); %x coordinate in the left front corner of the car
    y0 = y - w/2*cos(theta); %y coordinate in the left front corner of the car

    x1 = x - w/2*sin(theta); %y coordinate in the right front corner of the car
    y1 = y + w/2*cos(theta); %y coordinate in the right front corner of the car

    p = x - l*cos(theta); 
    q = y - l*sin(theta);

    x2 = p + w/2*sin(theta); %x coordinate in the left rear corner of the car
    y2 = q - w/2*cos(theta); %y coordinate in the left rear corner of the car

    x3 = p - w/2*sin(theta); %x coordinate in the right rear corner of the car
    y3 = q + w/2*cos(theta); %y coordinate in the right rear corner of the car

    %Draw the  center of the rear axle of an automobile
    plot (x, y, 'rs');
    axis([-20 160 0 100]);
    xlabel('x - cm');
    ylabel('y - cm');
    title('Parallel Parking ');
    hold on
    grid on;

    %Draw a diagram of the parking space
    h1 = line([-20 40], [60 60]);
    h2 = line([40 40], [60 85]);
    h3 = line([40 105], [85 85]);
    h4 = line([105 105], [60 85]);
    h5 = line([105 160], [60 60]);
    h6 = line([-20 160], [20 20]);
    set(h1, 'linewidth', 5, 'color', 'b');
    set(h2, 'linewidth', 5, 'color', 'b');
    set(h3, 'linewidth', 5, 'color', 'b');
    set(h4, 'linewidth', 5, 'color', 'b');
    set(h5, 'linewidth', 5, 'color', 'b');
    set(h6, 'linewidth', 5, 'color', 'b');

    %Draw the outline of the automobile body
    l0 = line([x0 x1], [y0 y1]);
    l1 = line([x1 x3], [y1 y3]);
    l2 = line([x2 x3], [y2 y3]);
    l3 = line([x0 x2], [y0 y2]);

    set(l0, 'linewidth', 4, 'color', 'm');
    set(l1, 'linewidth', 2, 'color', 'b');
    set(l2, 'linewidth', 4, 'color', 'g');
    set(l3, 'linewidth', 2, 'color', 'b');

end 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%A little backward%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% x=132; %汽车起点的 x 坐标
% y=45; %汽车起点的 y 坐标
% theta=0; %汽车车身起始倾角
phi=0; %max 45 degree
v=-5;


for i = 1:4
    pause(0.3);
    [x,y,theta,v]=state_update(x,y,theta,v,dt,phi,u);
    x_seq(i) = x; 
    y_seq(i) = y; 
    theta_seq(i) = theta; 
    phi_seq(i) = phi; 
    %fprintf('x = %f, y = %f, theta = %f, phi = %f, dx = %f, dy = %f\n', x, y,theta, phi, delta_x, delta_y);
    x0 = x + w/2*sin(theta); 
    y0 = y - w/2*cos(theta); 

    x1 = x - w/2*sin(theta); 
    y1 = y + w/2*cos(theta); 

    p = x - l*cos(theta);  
    q = y - l*sin(theta);

    x2 = p + w/2*sin(theta); 
    y2 = q - w/2*cos(theta); 

    x3 = p - w/2*sin(theta); 
    y3 = q + w/2*cos(theta); 

    plot (x, y, 'rs');
    axis([-20 160 0 100]);
    xlabel('x - cm');
    ylabel('y - cm');
    title('Parallel Parking ');
    hold on
    grid on;

    %画车身轮廓
    l0 = line([x0 x1], [y0 y1]);
    l1 = line([x1 x3], [y1 y3]);
    l2 = line([x2 x3], [y2 y3]);
    l3 = line([x0 x2], [y0 y2]);

    set(l0, 'linewidth', 4, 'color', 'm');
    set(l1, 'linewidth', 2, 'color', 'b');
    set(l2, 'linewidth', 4, 'color', 'g');
    set(l3, 'linewidth', 2, 'color', 'b');
     
end 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%The first sterring%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

phi=10.04; 


for i = 1:9
    pause(0.3);
    [x,y,theta,v]=state_update(x,y,theta,v,dt,phi,u);
    x_seq(i) = x; 
    y_seq(i) = y; 
    theta_seq(i) = theta; 
    phi_seq(i) = phi; 
    %fprintf('x = %f, y = %f, theta = %f, phi = %f, dx = %f, dy = %f\n', x, y,theta, phi, delta_x, delta_y);
    x0 = x + w/2*sin(theta); 
    y0 = y - w/2*cos(theta); 

    x1 = x - w/2*sin(theta); 
    y1 = y + w/2*cos(theta); 

    p = x - l*cos(theta);  
    q = y - l*sin(theta);

    x2 = p + w/2*sin(theta); 
    y2 = q - w/2*cos(theta); 

    x3 = p - w/2*sin(theta); 
    y3 = q + w/2*cos(theta); 

    plot (x, y, 'rs');
    axis([-20 160 0 100]);
    xlabel('x - cm');
    ylabel('y - cm');
    title('Parallel Parking ');
    hold on
    grid on;

    l0 = line([x0 x1], [y0 y1]);
    l1 = line([x1 x3], [y1 y3]);
    l2 = line([x2 x3], [y2 y3]);
    l3 = line([x0 x2], [y0 y2]);

    set(l0, 'linewidth', 4, 'color', 'm');
    set(l1, 'linewidth', 2, 'color', 'b');
    set(l2, 'linewidth', 4, 'color', 'g');
    set(l3, 'linewidth', 2, 'color', 'b');   
end 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%opposite direction parking%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

phi=39.9; 
v=-3;

for i = 1:6
    pause(0.3);
    [x,y,theta,v]=state_update(x,y,theta,v,dt,phi,u);
    x_seq(i) = x; 
    y_seq(i) = y; 
    theta_seq(i) = theta; 
    phi_seq(i) = phi; 
    %fprintf('x = %f, y = %f, theta = %f, phi = %f, dx = %f, dy = %f\n', x, y,theta, phi, delta_x, delta_y);
    x0 = x + w/2*sin(theta); 
    y0 = y - w/2*cos(theta); 

    x1 = x - w/2*sin(theta); 
    y1 = y + w/2*cos(theta); 

    p = x - l*cos(theta);  
    q = y - l*sin(theta);

    x2 = p + w/2*sin(theta); 
    y2 = q - w/2*cos(theta); 

    x3 = p - w/2*sin(theta); 
    y3 = q + w/2*cos(theta); 

    plot (x, y, 'rs');
    axis([-20 160 0 100]);
    xlabel('x - cm');
    ylabel('y - cm');
    title('Parallel Parking ');
    hold on
    grid on;

    l0 = line([x0 x1], [y0 y1]);
    l1 = line([x1 x3], [y1 y3]);
    l2 = line([x2 x3], [y2 y3]);
    l3 = line([x0 x2], [y0 y2]);

    set(l0, 'linewidth', 4, 'color', 'm');
    set(l1, 'linewidth', 2, 'color', 'b');
    set(l2, 'linewidth', 4, 'color', 'g');
    set(l3, 'linewidth', 2, 'color', 'b');   
end 

