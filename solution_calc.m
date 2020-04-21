function [v,phi]=solution_calc(x0,y0,theta0,N)
    Max = 50000; %increase the maximum number of iterations
    tol=0.000001;
    l = 26;
    x = zeros(N+1,Max); %state x
    y = zeros(N+1,Max); %state y
    theta = zeros(N+1,Max); %state theta
    u1 = zeros(N,Max+1); %acc
    u1(1:3,:)=5;
    u2 = zeros(N,Max+1); %phi
    l1 = zeros(N,Max); %lambda1
    l2 = zeros(N,Max); %lambda2
    l3 = zeros(N,Max); %lambda3
    J = zeros(Max); %objective / cost function

    n=1;
    conv=0;
    xd=70;
    yd=12;
    while (n<=Max)&(~conv)
        %initial state
        x(1,n)=x0;
        y(1,n)=y0;
        theta(1,n)=theta0;
        idx=0;
        %===Propagate the state forward
        for k=1:N
            [x(k+1,n),y(k+1,n),theta(k+1,n)]=state_update(x(k,n),y(k,n),theta(k,n),u1(k,n),u2(k,n));
            collision=detection(x(k+1,n),y(k+1,n),theta(k+1,n));
            if (collision==1 & idx==0)
                idx=k-2;
            end
        end
        J(n) = 0.5*((x(N+1,n)-xd)^2+(y(N+1,n)-yd)^2+500*theta(N+1,n)^2);

        l1(N,n)=x(N+1,n)-xd; %terminal co-state
        l2(N,n)=y(N+1,n)-yd;
        l3(N,n)=500*theta(N+1,n);
        %===Propagate the co-state backward
        for k=N-1:-1:1
            l1(k,n)=l1(k+1,n);
            l2(k,n)=l2(k+1,n);
            l3(k,n)=-l1(k+1,n)*u1(k,n)*sin(theta(k+1,n))+l2(k+1,n)*u1(k,n)*cos(theta(k+1,n))+l3(k+1,n);
        end

        %===Update the control input using gradient descent
        alpha = 0.001; %constant step size for simplicity
        for k=1:N
            u1(k,n+1) = u1(k,n)-alpha*(l1(k,n)*cos(theta(k,n))+l2(k,n)*sin(theta(k,n))+l3(k,n)*tan(u2(k,n))/l);
            u2(k,n+1) = u2(k,n)-alpha*(l3(k,n)*u1(k,n)*sec(u2(k,n))^2)/l;
            u1(k,n+1) = min(10,max(-10,u1(k,n+1))); %clipping the value of u to between -1 and 1
            u2(k,n+1) = min(.8,max(-.8,u2(k,n+1)));
        end
        if idx~=0
            xd=x(idx,n)-10*sin(theta(idx,n));
            yd=y(idx,n)+10*cos(theta(idx,n));
            [v_temp,phi_temp]=avoidance(x(idx,n),y(idx,n),theta(idx,n),xd,yd);
            u1(idx:idx+2,n+1)=v_temp;
            u2(idx:idx+2,n+1)=phi_temp;
        end
        if n>1
        conv = norm(J(n)-J(n-1))<tol;
        end
        n=n+1;
    end


    iter = n-1;%final iteration index
% 
%     figure(1)
%     plot(J(1:iter));xlabel('Iteration #');legend('Cost function (J)')
%     figure(2)
    plot(x(:,iter),y(:,iter),'r.-');xlabel('x');ylabel('y');grid on
    v=u1(:,iter);
    phi=u2(:,iter);
end