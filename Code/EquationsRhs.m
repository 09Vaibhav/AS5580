function [x_dot,y_dot,X_dot,xe_dot,ye_dot,Xe_dot,l_dot] = EquationsRhs(a,u,Tf)
global p
x = a(1);        % position of uav in x dirn
y = a(2);        % position of uav in y dirn
X = a(3);        % course angle of uav in xy plane
xe = a(4);       % error in position in x dirn
ye = a(5);       % error in position in y dirn
Xe = a(6);       % error in course angle 
l = a(7);        % parameter for virtual target position
% mu = 10*pi/180;   
mu = u(1);
[k,dD_dt] = kapa(l,p);
v = 0.1;
vq = (v/(1+sqrt(xe^2+ye^2)))/dD_dt;
g = 9.81;


x_dot = -Tf/2*(v*cos(X));
y_dot = -Tf/2*(v*sin(X));
X_dot = -Tf/2*(g*tan(mu)/(v));
xe_dot = -Tf/2*(ye*k*vq + v*cos(Xe) - vq*dD_dt);
ye_dot = -Tf/2*(-xe*k*vq + v*sin(Xe));
Xe_dot = -Tf/2*(g*tan(mu)/(v) - k*vq);
l_dot = -Tf/2*(vq);

end