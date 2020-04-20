clc;
clear;
close all;
l = 26; %wheelbase - mm
w = 15; %tread - mm
% u = 0;
v = 5;  %vehicle speed - m/s
dt =0.8; %Sampling interval - second

x=20; %The strating X coordinate of the vehicle
y=35; %The strating Y coordinate of the vehicle
theta=0; %Automobile body initial inclination
phi=0; %max 45 degree

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Go stright until find the parking place%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


figure; %draw the empty picture 
pause(1);
for i = 1:28
    pause(0.3);
    [x,y,theta]=state_update(x,y,theta,v,dt,phi);
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
    h1 = line([-20 160], [60 60]);
    h2 = line([-20 40], [20 20]);
    h3 = line([40 40], [0 20]);
    h4 = line([40 105], [0 0]);
    h5 = line([105 105], [0 20]);
    h6 = line([105 160], [20 20]);
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
[v,phi]=solution_calc();

for i = 1:15
    pause(0.3);
    [x,y,theta]=state_update(x,y,theta,v(i),1,phi(i));
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

    %»­³µÉíÂÖÀª
    l0 = line([x0 x1], [y0 y1]);
    l1 = line([x1 x3], [y1 y3]);
    l2 = line([x2 x3], [y2 y3]);
    l3 = line([x0 x2], [y0 y2]);

    set(l0, 'linewidth', 4, 'color', 'm');
    set(l1, 'linewidth', 2, 'color', 'b');
    set(l2, 'linewidth', 4, 'color', 'g');
    set(l3, 'linewidth', 2, 'color', 'b');
     
end 



