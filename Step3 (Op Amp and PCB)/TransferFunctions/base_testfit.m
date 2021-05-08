% load your data (text file with your two columns)
load baseAmp.txt;

% sample index, reducing of input to get single step instead of square wave
x = baseAmp(:,1);

%{
% plot data
figure(1)
plot(x,baseAmp(:,2)); hold off
%}

%timing interval declare
Ts = 0.000000005125;

% prepare data for tftest, TS is a random chosen sampling time
data = iddata([zeros(3,1);baseAmp(:,2)],[zeros(3,1);x],Ts);

% estimate system, factor 2 -> number of poles (variable as desired)
sys_base = tfest(data,1,0);

% Normalization - Multiply by sample rate
sys_base = sys_base*(Ts*2*100000/5);

% plot step response (factor 5 comes from input - 5V)
figure(4)
step(5*sys_base)