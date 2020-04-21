clc;
clear;
close all;
l = 26; %wheelbase - mm
w = 15; %tread - mm

x=20; %The strating X coordinate of the vehicle
y=35; %The strating Y coordinate of the vehicle
theta=0; %Automobile body initial inclination

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Go stright until find the parking place%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


figure; %draw the empty picture 
%Draw a diagram of the parking space
set(line([-20 160], [60 60]), 'linewidth', 5, 'color', 'b');
set(line([-20 40], [20 20]), 'linewidth', 5, 'color', 'b');
set(line([40 40], [0 20]), 'linewidth', 5, 'color', 'b');
set(line([40 100], [0 0]), 'linewidth', 5, 'color', 'b');
set(line([100 100], [0 20]), 'linewidth', 5, 'color', 'b');
set(line([100 160], [20 20]), 'linewidth', 5, 'color', 'b');
hold on
grid on;



for i = 1:10
    pause(0.1);
    [x,y,theta]=state_update(x,y,theta,5,0);

    [A,B,C,D]=edge(x,y,theta);

    axis([-20 160 0 100]);

    set(line([A(1) B(1)], [A(2) B(2)]), 'linewidth', 4, 'color', 'm');
    set(line([B(1) D(1)], [B(2) D(2)]), 'linewidth', 2, 'color', 'b');
    set(line([C(1) D(1)], [C(2) D(2)]), 'linewidth', 4, 'color', 'g');
    set(line([A(1) C(1)], [A(2) C(2)]), 'linewidth', 2, 'color', 'b');

end 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N=15;
[v,phi]=trajectory_calc(x,y,theta,N);
% [v,phi]=trajectory_opt(v,phi,x,y);

for i = 1:N
    pause(0.2);
    [x,y,theta]=state_update(x,y,theta,v(i),phi(i));
    

    collision=detection(x,y,theta);
    if collision
        disp('collision')
    end
    [A,B,C,D]=edge(x,y,theta);
    axis([-20 160 0 100]);
    set(line([A(1) B(1)], [A(2) B(2)]), 'linewidth', 4, 'color', 'm');
    set(line([B(1) D(1)], [B(2) D(2)]), 'linewidth', 2, 'color', 'b');
    set(line([C(1) D(1)], [C(2) D(2)]), 'linewidth', 4, 'color', 'g');
    set(line([A(1) C(1)], [A(2) C(2)]), 'linewidth', 2, 'color', 'b');
     
end 



