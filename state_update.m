function [x_next,y_next,theta_next]=state_update(x,y,theta,v,phi)
    l = 26; 
    dx=v*cos(theta);
    dy=v*sin(theta);
    dtheta=(v*tan(phi)/l);
    x_next=x+dx;
    y_next=y+dy;
    theta_next=theta+dtheta;
end