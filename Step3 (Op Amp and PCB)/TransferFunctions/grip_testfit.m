% load your data (text file with your two columns)
load gripAmp.txt;

% sample index, reducing of input to get single step instead of square wave
x = gripAmp(:,1);

%{
% plot data
figure(1)
plot(x,gripAmp(:,2)); hold off
%}

%timing interval declare
Ts = 0.00000001025;

% prepare data for tftest, TS is a random chosen sampling time
data = iddata([zeros(3,1);gripAmp(:,2)],[zeros(3,1);x],Ts);

% estimate system, factor 2 -> number of poles (variable as desired)
sys_grip = tfest(data,1,0);

% Normalization - Multiply by sample rate
sys_grip = sys_grip*(Ts*2*10000);


% plot step response (factor 5 comes from input - 5V)
figure(1)
step(5*sys_grip)
