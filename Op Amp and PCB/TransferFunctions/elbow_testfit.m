% load your data (text file with your two columns)
load elbowAmp.txt;

% sample index, reducing of input to get single step instead of square wave
x = elbowAmp(:,1);

%{
% plot data
figure(1)
plot(x,elbowAmp(:,2)); hold off
%}

%timing interval declare
Ts = 0.00000001025;

% prepare data for tftest, TS is a random chosen sampling time
data = iddata([zeros(3,1);elbowAmp(:,2)],[zeros(3,1);x],Ts);

% estimate system, factor 2 -> number of poles (variable as desired)
sys_elbow = tfest(data,1,0);

% Normalization - Multiply by sample rate
sys_elbow = sys_elbow*(Ts*2*10000);

% plot step response (factor 5 comes from input - 5V)
figure(3)
step(5*sys_elbow)