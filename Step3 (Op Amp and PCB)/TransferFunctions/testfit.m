% load your data (text file with your two columns)
load MyAmp.txt;

% sample index, reducing of input to get single step instead of square wave
x = MyAmp(:,1);

% plot data
figure(1)
plot(x,MyAmp(:,2)); hold off

%timing interval declare
Ts = 0.00000001025;

% prepare data for tftest, TS is a random chosen sampling time
data = iddata([zeros(3,1);MyAmp(:,2)],[zeros(3,1);x],Ts);

% estimate system, factor 2 -> number of poles (variable as desired)
sys = tfest(data,1,0);

% Normalization - Multiply by sample rate
sys = sys*(Ts*2*10000);

% plot step response (factor 5 comes from input - 5V)
figure(2)
step(5*sys)