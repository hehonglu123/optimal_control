
function [collision,y_d]=detection2(x,y,theta)

    [A,B,C,D]=edge(x,y,theta);
    if D(2)<0
        collision=1;
    else
        collision=0;
    end
    y_d=-D(2)+y;
end


