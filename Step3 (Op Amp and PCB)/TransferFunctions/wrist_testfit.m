% load your data (text file with your two columns)
load wristAmp.txt;

% sample index, reducing of input to get single step instead of square wave
x = wristAmp(:,1);

%{
% plot data
figure(1)
plot(x,wristAmp(:,2)); hold off
%}

%timing interval declare
Ts = 0.00000001025;

% prepare data for tftest, TS is a random chosen sampling time
data = iddata([zeros(3,1);wristAmp(:,2)],[zeros(3,1);x],Ts);

% estimate system, factor 2 -> number of poles (variable as desired)
sys_wrist = tfest(data,1,0);

% Normalization - Multiply by sample rate
sys_wrist = sys_wrist*(Ts*2*10000);

% plot step response (factor 5 comes from input - 5V)
figure(2)
step(5*sys_wrist)

