% Script with basic introduction to plotting graphs

% if we call other files before calling this
% plotting.m file, we can use variables, functions
% etc from those previous files. For example:
% >> plot(angles,sin_at_60_degree_interval);
% >> a % display vector a from vectors.m

clear

angs=linspace(0,2*pi,100); % 100 values evenly
% spaced between 0 and 2pi, nothing special about 100
y=sin(angs);


%%% PLOTTING %%%
plot(angs,y,'ro') % plot graph with red circles
title('Graph of y=sin(x)');
xlabel('Angle')
ylabel('Value')


%%% MULTIPLE GRAPHS %%%
plot(angs,y,':',angs,cos(angs),'-') % plot sin curve


%%% HOLD ON %%%
hold on % keep the graph, plot next graph on top
% using dotted line, plot cos curve using solid line

plot(angs,sin(angs)+cos(angs),'ro')


%%% ANNOTATIONS %%%
grid on % put grid on
legend('Sine','Cosine','Sin Plus Cosine') % adding
% names for the curves in plotting order


%%% MULTIPLE WINDOWS %%%
figure % next graph will be in a new window (figure 2)

plot(angs,tan(angs),'b')

figure(1) % go back to figure 1
axis([0,7,-5,5]) % manually scale the axis using the
%function axis([xmin,xmax,ymin,ymax])

figure % next graph will be in a new window (figure 3)
plot(angs,sin(angs)-cos(angs))
