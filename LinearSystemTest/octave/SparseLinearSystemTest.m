n = 10;

matrix = zeros(n,n);
matrix(1,1) = 1;
for i=2:(n-1)
	matrix(i,i-1) = -1;
	matrix(i,i) = 2;
	matrix(i,i+1) = -1;
end
matrix(n,n) = 1;

independent = zeros(n,1);
independent(n) = 1;

solution = matrix\independent;

format long
clc
solution
