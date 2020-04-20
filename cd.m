
function collision=cd(x,y,theta)
    p = x - l*cos(theta);  
    q = y - l*sin(theta);
    A=[x + w/2*sin(theta),y - w/2*cos(theta)];
    B=[x - w/2*sin(theta),y + w/2*cos(theta)];
    C=[p + w/2*sin(theta),q - w/2*cos(theta)];
    D=[p - w/2*sin(theta),q + w/2*cos(theta)];
end

function area=triangle(a,b,c)
    v1=[b-a,0];
    v2=[c-a,0];
    area=.5*norm(cross(v1,v2));
end


