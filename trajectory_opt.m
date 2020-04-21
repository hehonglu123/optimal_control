function [v,phi]=trajectory_opt(u1,u2,x0,y0)
    v=u1;
    phi=u2;
    collision1=1;
    collision2=1;
    while collision1==1 | collision2==1
        idx1=0;
        idx2=0;
        N=length(v);
        x = zeros(N+1); %state x
        y = zeros(N+1); %state y
        x(1)=x0;
        y(1)=y0;

        theta = zeros(N+1); %state theta
        for k=1:N
            [x(k+1),y(k+1),theta(k+1)]=state_update(x(k),y(k),theta(k),v(k),phi(k));
            
            collision1=detection(x(k+1),y(k+1),theta(k+1));
            if (collision1==1)
                idx1=k-2;
                break;
            end 
        end

        if idx1~=0
            xd=x(idx1)-10*sin(theta(idx1));
            yd=y(idx1)+10*cos(theta(idx1));
            [v_temp,phi_temp]=avoidance(x(idx1),y(idx1),theta(idx1),xd,yd);
            v(idx1:idx1+2)=v_temp;
            phi(idx1:idx1+2)=phi_temp;
        end
        for k=1:N
            [x(k+1),y(k+1),theta(k+1)]=state_update(x(k),y(k),theta(k),v(k),phi(k));
        end

        [v(idx1+3:end),phi(idx1+3:end)]=trajectory_calc2(x(idx1+3),y(idx1+3),theta(idx1+3),N-idx1-2);
        
        
        for k=1:N
            [x(k+1),y(k+1),theta(k+1)]=state_update(x(k),y(k),theta(k),v(k),phi(k));
            [collision2,yd]=detection2(x(k+1),y(k+1),theta(k+1));
            if (collision2==1)
                idx2=k-2;
                break;
            end 
        end
        if idx2~=0
            xd=x(idx2);
            [v_temp,phi_temp]=avoidance(x(idx2),y(idx2),theta(idx2),xd,yd);
            v(idx2:idx2+2)=v_temp;
            phi(idx2:idx2+2)=phi_temp;
        end
        [v(idx2+3:end),phi(idx2+3:end)]=trajectory_calc2(x(idx2+3),y(idx2+3),theta(idx2+3),N-idx2-2);
        
    end
    
  
end
