function [A,B,C,D]=edge(x,y,theta)
    l=26;
    w = 15;
    p = x - l*cos(theta);  
    q = y - l*sin(theta);
    A=[x + w/2*sin(theta),y - w/2*cos(theta)];
    B=[x - w/2*sin(theta),y + w/2*cos(theta)];
    C=[p + w/2*sin(theta),q - w/2*cos(theta)];
    D=[p - w/2*sin(theta),q + w/2*cos(theta)];
end