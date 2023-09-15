function [c,ceq,dc,dceq] = C_fun(Z)
global Nn Dmat

x = Z(1:Nn) ;
y = Z(Nn +1:2*Nn) ;
X = Z(2*Nn +1:3*Nn) ;
xe = Z(3*Nn+1:4*Nn) ;
ye = Z(4*Nn+1:5*Nn) ;
Xe = Z(5*Nn +1:6* Nn) ;
l = Z(6*Nn+1:7*Nn) ;
mu = Z(7*Nn+1:8*Nn) ;
Tf = Z(end);

%%
for i =1:Nn
[x_dot(i),y_dot(i),X_dot(i),xe_dot(i),ye_dot(i),Xe_dot(i),l_dot(i)] = ...
    EquationsRhs([x(i),y(i),X(i),xe(i),ye(i),Xe(i),l(i)],mu(i),Tf);
end

%%
ceq(1:Nn) = Dmat*x - x_dot';
ceq(Nn+1:2*Nn) = Dmat*y -y_dot';
ceq(2*Nn+1:3*Nn) = Dmat*X - X_dot';
ceq(3*Nn+1:4*Nn) = Dmat*xe - xe_dot';
ceq(4*Nn+1:5*Nn) = Dmat*ye - ye_dot';
ceq(5*Nn+1:6*Nn) = Dmat*Xe - Xe_dot';
ceq(6*Nn+1:7*Nn) = Dmat*l - l_dot';
ceq(7*Nn+1) = x(1)+0.1;
ceq(7*Nn+2) = y(1);
ceq(7*Nn+3) = X(1);
ceq(7*Nn+4) = l(1);
ceq(7*Nn+5) = l(end)-1;
ceq(7*Nn+6) = xe(1)+0.1;
ceq(7*Nn+7) = ye(1);
ceq(7*Nn+8) = Xe(1);

c = 0;
dc = 0;
dceq = 0;
end