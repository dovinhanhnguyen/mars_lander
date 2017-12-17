% Basic introduction to control statements

clear;


%%% IF...ELSE...END STATEMENT %%%
a=2
b=3
if a>b
  c=a-b
elseif a==b
  disp('a is equal to b')
else
  c=b-a
end

if c=~0 & sqrt(-1)==i % if c is equal to
% 'not 0' and sqrt(-1) is equal to i
  d=c+i
else
  d=0
end


%%% SWITCH STATEMENT %%%
e=0;
switch e
  case 0
    disp('e is zero');
  case 1
    disp('e is one');
  otherwise
    disp('e is not a binary digit');
end


%%% FOR LOOP %%%
for n=[1:5]
  abc=factorial(n) % abc is recalculated for each n
end

for n=[6:8]
  xyz(n)=factorial(n); % nth element of an unpredefined
  % vector xyz is set to n! without being displayed
end
xyz % display row vector xyz [0,0,0,0,0,720,5040,40320]


%%% WHILE LOOP %%%
m=80;
while m>10
  m=m/2
end
