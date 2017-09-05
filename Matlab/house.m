function [u,beta,alpha]= house(x)
% [u,beta,alpha] = house(x)
% H = eye(length(x)) - beta*u*u'
% u is scaled so that u(1)=1, so u(2:n) is the essential part.
[nr,nc] = size(x);
if nc~=1
    error('Input x to house is not a column vector.')
end
% scaling to avoid over(under)flow
m = max(abs(x));
x = x/m;
%
normx = norm(x); % normx >= 1.0
%
% get sign(x1)
if x(1)==0 % in matlab sign(0)=0
    signx1 = 1.0;
else
    signx1 = sign(x(1));
end
%
alpha = signx1*normx; % |alpha| >= 1.0
%
u = x;
u(1)= u(1) + alpha; % |u(1)| >= 1.0
beta = 1/(alpha'*u(1));
%
% make u(1)=1, so u(2:n) is the essential part of u
beta = beta*(u(1)*u(1)'); % update beta
u = u/u(1);
% scale back
alpha = m*alpha;