function [k,dD_dt] = kapa(l,p)

global traj_model

if traj_model ==1 % Circle
    r = p(1);
    k = 2*pi;
    dD_dt = 2*pi*r;
elseif traj_model == 2 % Spiral 
    a=p(1); % Constant radius part
    b=p(2); % Coeff of variable radius
    c=p(3); % No of spirals
    k = 2*pi*c*(4*pi^2*b^2*c^2*l^2+4*pi*a*b*c*l+2*b^2+a^2)/...
        (4*pi^2*b^2*c^2*l^2+4*pi*a*b*c*l+b^2+a^2);
    dD_dt = 2*pi*c*sqrt(((2*pi*b*c*l+a)*sin(2*pi*c*l)-b*cos(2*pi*c*l))^2+((2*pi*b*c*l+a)*cos(2*pi*c*l)+b*sin(2*pi*c*l))^2);
elseif traj_model == 3 % Elliptical
    a=p(1);
    b=p(2);
    k = (2*pi*a*b*(csc(2*pi*l))^2)/(b^2*(cot(2*pi*l))^2+a^2);
    dD_dt = 2*pi*sqrt(a^2*(sin(2*pi*l))^2+b^2*(cos(2*pi*l))^2);
elseif traj_model == 4 % Straight line
    a=p(1);
    b=p(2);
    k = 0;
    dD_dt = a*sqrt(1+b^2);
end 

end