# Simple harmonic spring with using Euler method

clear;
graphics_toolkit("gnuplot");

# Constants
m = 1;
dt = 0.1;
k = 1;
tmax = 100;

# Initial conditions
t = 0; x = 0; v = 1;
imax = (tmax-t)/dt;
trx = zeros(imax,1);
trv = zeros(imax,1);
trt = zeros(imax,1);

# Loop over time
for i = 1:imax
	trx(i) = x;
	trv(i) = v;
	trt(i) = t;
	a = -k*x/m;
	x = x+dt*v;
	v = v+dt*a;
	t = t+dt;
end

# Plot position and velocity against time
figure;
plot(trt,trx,'r',trt,trv,'g');
grid on;
title('Euler Method');
xlabel(dt);
legend('Position','Velocity');
