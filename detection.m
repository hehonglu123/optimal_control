
function collision=detection(x,y,theta)

    [A,B,C,D]=edge(x,y,theta);
    E=[105,20];
    area1=triangle(A,E,B);
    area2=triangle(B,E,C);
    area3=triangle(C,E,D);
    area4=triangle(D,E,A);
    if area1+area2+area3+area4<390
        collision=1;
    else
        collision=0;
    end
end

function area=triangle(a,b,c)
    v1=[b-a,0];
    v2=[c-a,0];
    area=.5*norm(cross(v1,v2));
end


