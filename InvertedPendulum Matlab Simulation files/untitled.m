clc;
close all;
clear all;
% Define the numerator and denominator of the transfer function
numerator = [1];                  % Coefficients of the numerator: K (constant 1 for simplicity)
denominator = [1 15 50 0];        % Coefficients of the denominator: s(s+5)(s+10) = s^3 + 15s^2 + 50s

% Create the transfer function
sys = tf(numerator, denominator);

% Plot the root locus
rlocus(sys);
grid on;                         % Add grid for better readability
title('Root Locus of the Transfer Function K/s(s+5)(s+10)');
xlabel('Real Axis');
ylabel('Imaginary Axis');
