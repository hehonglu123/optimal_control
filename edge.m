function [A,B,C,D]=edge(x,y,theta)
    l=26;
    w = 15;
%     p = x - l*cos(theta);  
%     q = y - l*sin(theta);
    A=[x - w/2*sin(theta)+l/2*cos(theta),y + w/2*cos(theta)+l/2*sin(theta)];
    B=[x + w/2*sin(theta)+l/2*cos(theta),y - w/2*cos(theta)+l/2*sin(theta)];
    C=[x - w/2*sin(theta)-l/2*cos(theta),y + w/2*cos(theta)-l/2*sin(theta)];
    D=[x + w/2*sin(theta)-l/2*cos(theta),y - w/2*cos(theta)-l/2*sin(theta)];
end