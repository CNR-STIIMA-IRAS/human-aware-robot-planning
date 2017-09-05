function [ U ] = unitarand( m,n )
%RANDROTMATRIX initialize a random diagonal unitary matrix
x= (ones(m,1) - 2*rand(m,1));
U= diag(sign(x));
%
for i=m:-1:max(2,m-n+1)
    % generate an i-dim. random vector with
    % uniform distribution on [-1, 1]
    x= (ones(i,1) - 2*rand(i,1));
    % generate Householder matrix
    [u,beta,alpha]= house(x);
    % accumulate Householder transformations
    U(:,m-i+1:m)= U(:,m-i+1:m) - (U(:,m-i+1:m)*u)*(beta*u');
end
U= U(:,1:n);

end

