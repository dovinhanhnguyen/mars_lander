% Script with basic introduction to vectors

clear; % clear everything


%%% VECTORS %%%
% ';' tells Octave to not display the result
a=[1,4,5]; % this is a row vector
b=[4;7;10]; % this is a column vector
c=[a,6]; % append 6 to the row vector a
d=[b;12;14]; % append 12 and 14 to the column vector b

a % display a, which is row [1,4,5]
b % display b, which is column [4;7;10]
c % display c, which is row [1,4,5,6]
d % display d, which is column [4;7;10;12;14]


%%% COLON NOTATION %%%
e=2:6; % row vector [2,3,4,5,6]
f=2:0.3:3; % row vector [2.0,2.3,2.6,2.9]


%%% COMBINATION OF NOTATIONS %%%
f2=[1:2:6,-1,0] % row [1,3,5,-1,0]


%%% DISPLAYING LARGE VECTORS AND MATRICES %%%
% 'more off' to turn off the pagination feature
g=1:1000; % row vector [1,2,3,...,1000]


%%% VECTOR-CREATING FUNCTIONS %%%
h=zeros(10,1) % column vector [0;0;...ten times...;0]
i=ones(1,10) % row [1,1,...ten times...,1]


%%% ACCESSING ELEMENTS %%%
f2(4) % gives fourth element of f2, which is -1
g(3:5) % gives third, fourth, fifth elements of g
g(40:2:52) % gives 40th,42nd,44th,...,52nd


%%% VECTOR MATHS %%%
j=a*2; % multiplies the whole vector a by 2
k=i/2; % divides the whole vector i by 2
j
k

% element-by-element operations, vectors must have
% same size and shape
l=[2,4,6,8,10]
m=[1,2,3,4,5]
l_plus_m=l+m
l_minus_m=l-m
n=l.*m % which gives row [2,8,18,32,50]
o=l./m % which gives row [2,2,2,2,2]
m.^2 % raise m to power of 2, gives row [1,4,9,16,25]
2.^m % raise 2 to power in b, gives row [2,4,8,16,32]

% passing vector to a function
angles=[0:pi/3:2*pi]
sin_at_60_degree_interval=sin(angles)
