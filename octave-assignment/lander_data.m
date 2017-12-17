# Script to handle data from lander autopilot

clear;
graphics_toolkit("gnuplot");

data = csvread ("lander_data.csv");

# Plot actual and target descent rate against altitude
figure;
plot(data(:,3),data(:,4),'r',data(:,3),data(:,5),'b');
grid on;
title('Actual vs Target Descent Rate');
xlabel('Altitude above Mars surface (m)');
ylabel('Descent rate (m/s)');
legend("Actual", "Target");
