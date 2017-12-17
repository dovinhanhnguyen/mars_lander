# Straight down descent using Verlet method

clear;
graphics_toolkit("gnuplot");

# Constant throughout
m = 1; # mass of lander
dt = 1; # time step
G = 6.67*10^(-11); # universal gravitational constant
M = 6.42*10^(23); # mass of Mars

# Initial conditions
t = 0;
x = [4000000,0,0];
v = [0,0,0];
trx = x;
trv = v;
trt = zeros(1,1);

# First Iteration
a = (-G*M/(norm(x)^3))*x; # unit vector so ^3
x = x+dt*v+0.5*(dt^2)*a;
v = v+dt*a;
t = t+dt;

# Loop over time until x=0
while x(1,1) >= 0
	trx = [trx;x]; # add x as a new row to matrix trx
	trv = [trv;v]; # add v as a new row to matrix trv
	trt = [trt;t]; # add t as a new row to matrix trt
	a = (-G*M/(norm(x)^3))*x; # unit vector so ^3
	x = 2*x-trx(rows(trx)-1,:)+(dt^2)*a;
	v = v+dt*a;
	t = t+dt;
end

# Plot altitude wrt time
figure;
plot(trt,trx(:,1)); 
grid on;
title('Verlet Position - Straight Down Descent');
xlabel(dt);

# Plot velocity wrt time
figure;
plot(trt,trv(:,1)); 
grid on;
title('Verlet Velocity - Straight Down Descent');
xlabel(dt);

# Plot trajectory in 3d space
figure;
plot3(trx(:,1),trx(:,2),trx(:,3));
grid on;
title('Verlet Trajectory - Straight Down Descent');
xlabel(dt);
