function [x_next,y_next,theta_next]=state_update(x,y,theta,v,dt,phi)
    l = 26; 
    dx=v*cos(theta);
    dy=v*sin(theta);
    dtheta=(v*tan(phi)/l);
    x_next=x+dx*dt;
    y_next=y+dy*dt;
    theta_next=theta+dtheta*dt;
%     v_next=v+u*dt;
end