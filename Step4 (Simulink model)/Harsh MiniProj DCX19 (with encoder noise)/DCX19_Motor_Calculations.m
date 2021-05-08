clear
%----------------DCX19S (34 mm length) Shoulder Motor (Nominal 24V) --------------
%Parameters used for calculation of B (damping factor) 
Nominal_V = 24;              %V
NL_current = 13.2*10^-3;     %A
NL_speed = 6350*(2*pi/60);   %rad/s - speed in rpm converted to rad/s

%Parameters directly fed into the motor Model

Km = 35.6*10^-3;                               % (Nm/A) Torque constant 
Kb = 1/(268*(2*pi/60));                            % [(rad/s/V) converted from rpm/V]....Back-EMF constant/Speed constant
R  = 22.8;                                     % Armature resistance - ohms
L  = 1.320*10^-3;                              % Armature inductance - H (converted from mH)
J  = 2.72*10^-7;                                %Currently Missing arm inertia
B  = (NL_current*Km)/NL_speed; 

G = tf(Km,[L*J (R*J +L*B) R*B]);
H = tf(Kb,1);
MotorTF = feedback(G,H);       %Motor Transfer Function

%Op Amp (A1/(B1s + B2))     %First order Multisim TF response
A1 = 115.3;
B1 = 1;
B2 = 23.65;
OPAMP = tf([A1] , [B1 B2]);
INTEGRATOR_TF = tf([1],  [1 0]);
SystemTF = MotorTF*OPAMP*INTEGRATOR_TF;
[z,p,k] = zpkdata(SystemTF);    %obtain zeros and poles from TF
filter_pole = 50;
system_poles = [-17065.6719111424;-209.653442554962;-23.6500000000000; 0]; %from zpk data
system_zeros = [];  %from zpk data

%----------Obtaining the two zeros, filter pole, and K value to determine PID gains Kp, Ki, Kd----%

%Designing zeros to have half the magnitude of nearest pole to jw axis and starting angle 45 
%tune zeros closer to jw axis for more stability and also reduce K to stabilise.

zeros_mag = abs(system_poles(3)/2);
zeros_angle = 45;
zeros_angle_rad = deg2rad(zeros_angle);
[Gm,Pm,Wcg,Wcp] = margin(SystemTF); %Gm is obtained in pure value and not dB.
Ku = Gm; %not in dB 

%z's, P (N) and K
z1 = zeros_mag*exp(1i*zeros_angle_rad); %Note that this is not (pi - angle) because we want POSITIVE zero values
z2 = zeros_mag*exp(1i*-zeros_angle_rad); %Similarly not pi + angle
K = 0.1*Ku;

%New root locus and GM/PM plots to see improvement
s = tf('s');
PIDTF = K*((s+z1)*(s+z2))/(s*(s+filter_pole)) %Kp + Ki/s + Kd*(filter_pole*s)/(s+filter_pole);
PID_Filtered_TF = PIDTF*SystemTF;

%PID gains
Ki = ((z1*z2)*K)/filter_pole;
Kp =  (K*(z1 + z2) - Ki)/filter_pole;
Kd = (K - Kp)/filter_pole;

figure(1);
subplot(2,1,1)
rlocus(SystemTF);
subplot(2,1,2)
rlocus(PID_Filtered_TF);
figure(2);
title('Increase in GM and PM based on initial tuning calculations');
subplot(2,1,1)
margin(SystemTF);
subplot(2,1,2)
margin(PID_Filtered_TF);


%optical encoder resolution
CPR = 128; %counts per revolution arbitrary number for now
res = (2*pi)/CPR;

desired_angle_vectors = [pi/2 -1.57 1.57/2];
ept = zeros(1,8);
for i = 1:8
        ept(i) = 5000*exp(-5000*i*0.0001)
    end
ept = ept/(0.0001*sum(ept));
