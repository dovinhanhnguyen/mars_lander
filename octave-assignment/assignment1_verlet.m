# Simple harmonic spring using Verlet method

clear;
graphics_toolkit("gnuplot");

# Constants
m = 1;
dt = 1;
k = 1;
tmax = 100;

# Initial conditions
t = 0; x = 0; v = 1;
imax = (tmax-t)/dt;
trx = zeros(imax,1);
trv = zeros(imax,1);
trt = zeros(imax,1);

# First iteration
trx(1,1) = t;
trx(1,1) = x;
a = -k*x/m;
x = x+v*dt+0.5*a*(dt)^2;
v = v+a*dt;
t = t+dt;

# Loop over time
for i = 2:imax
	trx(i) = x;
	trv(i) = v;
	trt(i) = t;
	a = -k*x/m;
	x = 2*trx(i)-trx(i-1)+a*(dt)^2;
	v = v+a*dt;
	t = t+dt;
end

# Plot position against time
figure;
plot(trt,trx);
grid on;
title('Verlet Position');
xlabel(dt);

# Plot velocity against time
figure;
plot(trt,trv);
grid on;
title('Verlet Velocity');
xlabel(dt);
