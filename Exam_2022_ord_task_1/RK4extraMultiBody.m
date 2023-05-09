function [tspan,Y,Yd,Ydd] = RK4extraMultiBody(f,tspan,dt,yini,Nvar,varargin)
% [tspan,Y,Yd,Ydd] = RK4(f,tspan,dt,yini,Nvar,var1,var2...var10)
% 
% INPUT
% f         -> name of function
% tspan     -> interval of integration linspace(tmin:dt:tmax)
% dt        -> increment
% yini      -> initial values
% Nvar      -> number of equations that is integrated
% var1,var2 -> extra variables that is passed on to the function f
% 
% OUTPUT
% tspan       -> interval of integration linspace(tmin:dt:tmax)
% Y           -> Position
% Yd          -> Velocity
% Ydd         -> Acceleration

n = length(tspan);
m = length(yini);
p = 3*Nvar;
q = 2*Nvar;
r = 2*Nvar+1;
s = Nvar+1;
y = yini;
Y(1:n,1:Nvar) = 0;
Yd(1:n,1:Nvar) = 0;
Ydd(1:n,1:Nvar) = 0;
Y(1,1:Nvar) = yini(1:Nvar,1)';
Yd(1,1:Nvar) = yini(s:q,1)';

for i=1:(n-1)
    temp = feval(f,tspan(i),y,varargin{:});
    Ydd(i,1:Nvar) = temp(s:q,1)';
    k1 = dt*temp;
%     k1 = dt*feval(f,tspan(i),y,varargin{1}(:,:));
    k2 = dt*feval(f,tspan(i)+dt/2,y+k1/2,varargin{:});
    k3 = dt*feval(f,tspan(i)+dt/2,y+k2/2,varargin{:});
    k4 = dt*feval(f,tspan(i)+dt,y+k3,varargin{:});
    y = y + k1/6 + k2/3 +k3/3 + k4/6;
    Y(i+1,1:Nvar) = y(1:Nvar,1)';
    Yd(i+1,1:Nvar) = y(s:q,1)';
end
temp = feval(f,tspan(n),y,varargin{:});
Ydd(n,1:Nvar) = temp(s:q,1)';













