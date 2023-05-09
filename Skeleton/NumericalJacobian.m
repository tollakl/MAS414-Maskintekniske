function b = NumericalJacobian(FunFcn,x,p)
% NewtonRaphson(FunFcn,x,p,tol,relax)
% 
% NewtonRaphson Solves a set of functions (linear or non linear) by means of
% Newton-Raphson iteration. 
% FunFcn is a string holding name of function
% x      is initial guess on variables
% p      is a set of fixed parameters
% tol    is the tolerance to be met in iteration
% relax  is the relaxation factor for the change in the coordinate vector
% 
% Michael Rygaard Hansen
% Aalborg University, 1999
%
% modified by
% Niels L. Pedersen 2003
% 
% Explanation added by
% Morten Kjeld Ebbesen 2005


% Get rank of problem
n = length(x);

% Set perturbation
pert = 1e-10;

% Initialize counter
i = 0;

% Initial residual vector computation
b = feval(FunFcn, x, p);
n2 = length(b);

  % Generate jacobian
  for j=1:n
    temp=x(j);  
    if x(j)==0  
      x(j) = x(j)+pert;
      delta=pert;
    else
      x(j) = x(j)*(1.0+pert);
      delta = pert*temp;
    end  
    bPert = feval(FunFcn, x, p);
    x(j) = temp;
    for k=1:n2
      jacob(k,j)=(bPert(k)-b(k))/delta;
    end
  end  

b = jacob;
