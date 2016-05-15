clear
clc
measured_x = 1:100;
measured_y = 1:100;
measured_z = 1:100;


% Measurement matrix
H = [1, 0, 0;
     0, 1, 0;
     0, 0, 1];
% Covariance of process noise
Q = [0.1,  0,   0;
      0,  0.01, 0;
      0,    0,   0.1];
% Covariance matrix representing errors in state estimates before update
M = 10*eye(3);
% Covariance matrix representing errors in state estimates (i.e. variance of
% truth minus estimate)
P = 10*eye(3);
% Covariance of measurement noise
R = [1,    0,   0;
      0,   1,   0;
      0,    0,  1];
% state vector
x = [0, 0, 0]';
% Pcc vector
Pcc = [0, 0, 0, 0]';

hist_x = zeros(length(measured_x), 3); 
hist_P = zeros(length(measured_x), 3); 

for idx=1:length(measured_x)
    
    %% STEP 1: time update -> prediction 
    x_old = x;
    M = P + Q;
    
    %% STEP 2: update with local sensor measurement
    % measurement vector
    z = [measured_x(idx); measured_y(idx); measured_z(idx)];
    % measurement update -> correction
    K = (M*H')/(H*P*H' + R);
    x = x_old + K*(z - H*x_old);
    P = M - K*H*M;
    
    % save history of x
    hist_x(idx, 1) = x(1);
    hist_x(idx, 2) = x(2);
    hist_x(idx, 3) = x(3);
    
    % save history of P_star
    hist_P(idx, 1) = P(1,1);
    hist_P(idx, 2) = P(2,2);
    hist_P(idx, 3) = P(3,3);
end

% visualization 
figure(4);
set(gcf,'color','w');
set(gca, 'Box', 'off');

ha(1) = subplot(3,1,1);
plot(1:length(measured_x),measured_x, '.r');
hold on; plot(1:length(measured_y),measured_y, '.b');
hold on; plot(1:length(measured_z),measured_z, '.g');

ha(2) = subplot(3,1,2);
plot(1:length(hist_x),hist_x(:,1), '.r');
hold on; plot(1:length(hist_x),hist_x(:,2), '.m');
hold on; plot(1:length(hist_x),hist_x(:,3), '.b');
title('Kalman filter output');

ha(4) = subplot(3,1,3);
plot(1:length(hist_x),hist_P(:,1), '.r');
hold on; plot(1:length(hist_x),hist_P(:,2), '.m');
hold on; plot(1:length(hist_x),hist_P(:,3), '.b');
title('Covariance Kalman filter');



