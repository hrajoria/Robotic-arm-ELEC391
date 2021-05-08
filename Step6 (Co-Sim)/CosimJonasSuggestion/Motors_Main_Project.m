clear
%---------------Motor Data Vectors (Gripper -> Wrist -> Elbow -> Sholder)----------------
%Parameters used for calculation of B (damping factor) 
Nominal_V = [12 12 12 12];              %V
NL_current = [2.74 2.74 23.2 274]*10^-3;     %A
NL_speed = [12900 12900 10300 7120]*(2*pi/60);   %rad/s - speed in rpm converted to rad/s

%Parameters directly fed into the motor Model

Km = [8.71 8.71 10.9 15.6]*10^-3;                               % (Nm/A) Torque constant 
Kb = 1./([1100 1100 872 612]*(2*pi/60));                            % [(rad/s/V) converted from rpm/V]....Back-EMF constant/Speed constant
R  = [92.2 92.2 5.31 0.108];                                     % Armature resistance - ohms
L  = [0.276 0.276 0.227 0.034]*10^-3;                              % Armature inductance - H (converted from mH)
J  = [0.035 0.035 0.966 77.6]*10^-7;                                %Currently Missing arm inertia
B  = (NL_current.*Km)./NL_speed; 


%----------------DCX8 (16 mm length) Gripper Motor (Nominal 12V) --------------

%Parameters used for calculation of B (damping factor) 
GNominal_V = Nominal_V(1);              %V
GNL_current = NL_current(1);     %A
GNL_speed = NL_speed(1);   %rad/s - speed in rpm converted to rad/s

%Parameters directly fed into the motor Model

GKm = Km(1);                               % (Nm/A) Torque constant 
GKb = Kb(1);                            % [(rad/s/V) converted from rpm/V]....Back-EMF constant/Speed constant
GR  = R(1);                                     % Armature resistance - ohms
GL  = L(1);                              % Armature inductance - H (converted from mH)
GJ  = J(1);                                %Currently Missing arm inertia
GB  = B(1); 

GG = tf(Km(1),[L(1)*J(1) (R(1)*J(1) +L(1)*B(1)) R(1)*B(1)]);
GH = tf(Kb(1),1);
GMotorTF = feedback(GG,GH);       %Motor Transfer Function

%Op Amp (A1/(B1s + B2))     %First order Multisim TF response
GA1 = 7.311*10^5;
GB1 = 1;
GB2 = 3.107*10^5;
GOPAMP = tf([GA1], [GB1 GB2]);
INTEGRATOR_TF = tf([1],  [1 0]);
SystemTF1 = GMotorTF*GOPAMP*INTEGRATOR_TF;
[z,p,k] = zpkdata(SystemTF1);    %obtain zeros and poles from TF

system_poles_gripper = p{:,1}; %from zpk data
system_zeros_gripper = z{:,1}; %from zpk data


%----------------DCX8 (16 mm length) Wrist Motor (Nominal 12V) --------------

%Parameters used for calculation of B (damping factor) 
WNominal_V = Nominal_V(2);              %V
WNL_current = NL_current(2);     %A
WNL_speed = NL_speed(2);   %rad/s - speed in rpm converted to rad/s

%Parameters directly fed into the motor Model

WKm = Km(2);                               % (Nm/A) Torque constant 
WKb = Kb(2);                            % [(rad/s/V) converted from rpm/V]....Back-EMF constant/Speed constant
WR  = R(2);                                     % Armature resistance - ohms
WL  = L(2);                              % Armature inductance - H (converted from mH)
WJ  = J(2);                                %Currently Missing arm inertia
WB  = B(2); 

WG = tf(Km(2),[L(2)*J(2) (R(2)*J(2) +L(2)*B(2)) R(2)*B(2)]);
WH = tf(Kb(2),1);
WMotorTF = feedback(WG,WH);       %Motor Transfer Function

%Op Amp (A1/(B1s + B2))     %First order Multisim TF response
WA1 = 7.311*10^5;
WB1 = 1;
WB2 = 3.107*10^5;
WOPAMP = tf([WA1] , [WB1 WB2]);
INTEGRATOR_TF = tf([1],  [1 0]);
SystemTF2 = WMotorTF*WOPAMP*INTEGRATOR_TF;
[z,p,k] = zpkdata(SystemTF2);    %obtain zeros and poles from TF

system_poles_wrist = p{:,1}; %from zpk data
system_zeros_wrist = z{:,1}; %from zpk data


%----------------DCX14L (35.6 mm length) Elbow Motor (Nominal 12V) --------------

%Parameters used for calculation of B (damping factor) 
ENominal_V = Nominal_V(3);              %V
ENL_current = NL_current(3);     %A
ENL_speed = NL_speed(3);   %rad/s - speed in rpm converted to rad/s

%Parameters directly fed into the motor Model

EKm = Km(3);                               % (Nm/A) Torque constant 
EKb = Kb(3);                            % [(rad/s/V) converted from rpm/V]....Back-EMF constant/Speed constant
ER  = R(3);                                     % Armature resistance - ohms
EL  = L(3);                              % Armature inductance - H (converted from mH)
EJ  = J(3);                                %Currently Missing arm inertia
EB  = B(3); 

EG = tf(Km(3),[L(3)*J(3) (R(3)*J(3) +L(3)*B(3)) R(3)*B(3)]);
EH = tf(Kb(3),1);
EMotorTF = feedback(EG,EH);       %Motor Transfer Function

%Op Amp (A1/(B1s + B2))     %First order Multisim TF response
EA1 = 7.371*10^5;
EB1 = 1;
EB2 = 3.124*10^5;
EOPAMP = tf([EA1] , [EB1 EB2]);
INTEGRATOR_TF = tf([1],  [1 0]);
SystemTF3 = EMotorTF*EOPAMP*INTEGRATOR_TF;
[z,p,k] = zpkdata(SystemTF3);    %obtain zeros and poles from TF

system_poles_elbow = p{:,1}; %from zpk data
system_zeros_elbow = z{:,1}; %from zpk data


%----------------DCX8 (16 mm length) Shoulder Motor (Nominal 12V) --------------

%Parameters used for calculation of B (damping factor) 
SNominal_V = Nominal_V(4);              %V
SNL_current = NL_current(4);     %A
SNL_speed = NL_speed(4);   %rad/s - speed in rpm converted to rad/s

%Parameters directly fed into the motor Model

SKm = Km(4);                               % (Nm/A) Torque constant 
SKb = Kb(4);                            % [(rad/s/V) converted from rpm/V]....Back-EMF constant/Speed constant
SR  = R(4);                                     % Armature resistance - ohms
SL  = L(4);                              % Armature inductance - H (converted from mH)
SJ  = J(4);                                %Currently Missing arm inertia
SB  = B(4); 

SG = tf(Km(4),[L(4)*J(4) (R(4)*J(4) +L(4)*B(4)) R(4)*B(4)]);
SH = tf(Kb(4),1);
SMotorTF = feedback(SG,SH);       %Motor Transfer Function

%Op Amp (A1/(B1s + B2))     %First order Multisim TF response
SA1 = 1.509*10^5;
SB1 = 1;
SB2 = 6.291*10^5;
SOPAMP = tf([SA1] , [SB1 SB2]);
INTEGRATOR_TF = tf([1],  [1 0]);
SystemTF4 = SMotorTF*SOPAMP*INTEGRATOR_TF;
[z,p,k] = zpkdata(SystemTF4);    %obtain zeros and poles from TF

system_poles_shoulder = p{:,1}; %from zpk data
system_zeros_shoulder = z{:,1}; %from zpk data

%--------PID Gains for each motor------------

%Gripper: 

%Designing zeros to have half the magnitude of nearest pole to jw axis and starting angle 45 
%tune zeros closer to jw axis for more stability and also reduce K to stabilise.

zeros_mag = abs(system_poles_gripper(4)/2);
zeros_angle = 45;
zeros_angle_rad = deg2rad(zeros_angle);
[Gm,Pm,Wcg,Wcp] = margin(SystemTF1); %Gm is obtained in pure value and not dB.
Ku = Gm; %not in dB 

%z's, P (N) and K
z1 = zeros_mag*exp(1i*zeros_angle_rad); %Note that this is not (pi - angle) because we want POSITIVE zero values
z2 = zeros_mag*exp(1i*-zeros_angle_rad); %Similarly not pi + angle
filter_pole = 50;
K = 0.3*Ku;

%PID gains
Ki = ((z1*z2)*K)/filter_pole;
Kp =  (K*(z1 + z2) - Ki)/filter_pole;
Kd = (K - Kp)/filter_pole;

%Calculations similar for other motors.....


% -------Encoder Code for sensor feedback estimation-----------
%optical encoder resolution
CPR = 128; %counts per revolution arbitrary number for now
res = (2*pi)/CPR;


%----input angles(Make this an Inverse kin function later)-----------------------------------------------------
%marshmallow 1
degree_vectors_m1 = [0 150 -150 120]; % gripper wrist elbow shoulder 
desired_angle_vectors_m1 = deg2rad(degree_vectors_m1);
degree_vectors_drop1 = [0 -40 40 120]; % gripper wrist elbow shoulder 
desired_angle_vectors_drop1 = deg2rad(degree_vectors_drop1);

%marshmallow 2
degree_vectors_m2 = [0 40 -40 25]; % gripper wrist elbow shoulder 
desired_angle_vectors_m2 = deg2rad(degree_vectors_m2);
degree_vectors_drop2 = [0 -40 40 25]; % gripper wrist elbow shoulder 
desired_angle_vectors_drop2 = deg2rad(degree_vectors_drop2);

%marshmallow 3
degree_vectors_m3 = [0 -210 210 20]; % gripper wrist elbow shoulder 
desired_angle_vectors_m3 = deg2rad(degree_vectors_m3);
degree_vectors_drop3 = [0 -40 40 10]; % gripper wrist elbow shoulder 
desired_angle_vectors_drop3 = deg2rad(degree_vectors_drop3);


desired_angle_vectors_origin = [0 0 0 0]; %resting position - implemented after every time a marshmallow is dropped in chute

%------Code to see improvement in stability from PID with Gain and phase margin (for control report)---------------
%New root locus and GM/PM plots to see improvement
% s = tf('s');
% PIDTF = Kp + Ki/s + Kd*(filter_pole*s)/(s+filter_pole);
% PID_Filtered_TF = PIDTF*SystemTF1;
% 
% 
% subplot(2,2,1)
% rlocus(SystemTF1);
% subplot(2,2,2)
% margin(SystemTF1);
% subplot(2,2,3)
% rlocus(PID_Filtered_TF);
% subplot(2,2,4)
% margin(PID_Filtered_TF);
% sgtitle('Increase in GM and PM based on initial tuning calculations');

% angle_shoulder = 0.523;
% [ elbow_coordinate_x, elbow_coordinate_y] = fcn(angle_shoulder)
% 
% function [elbow_coordinate_x, elbow_coordinate_y] = fcn(angle_shoulder)
% angle_shoulder_deg = rad2deg(angle_shoulder);
% 
% % If the input angle is negative, make it positive for analysis by adding 360
% if angle_shoulder_deg < 0
%     angle_shoulder_deg = 360 + angle_shoulder_deg;
% end
% 
% %conditional statements for obtaining the coordinates of the elbow and wrist motors from the angle traversed
% if angle_shoulder_deg < 90
%     elbow_coordinate_x = 5*cosd(angle_shoulder_deg);
%     elbow_coordinate_y = 5*sind(angle_shoulder_deg);
%     
% elseif 90 < angle_shoulder_deg && angle_shoulder_deg < 180 %2nd quadrant - x coordinates are negative
%     elbow_coordinate_x = -5*cosd(180 - angle_shoulder_deg);
%     elbow_coordinate_y = 5*sind(180 - angle_shoulder_deg);
%         
% elseif 180 < angle_shoulder_deg && angle_shoulder_deg < 270 %2nd quadrant - x coordinates are negative
%     elbow_coordinate_x = -5*cosd(angle_shoulder_deg - 180);
%     elbow_coordinate_y = 5*sind(angle_shoulder_deg - 180);
%    
% else                                                        %4th quadrant - x coordinates are negative
%     elbow_coordinate_x = 5*cosd(360 - angle_shoulder_deg);
%     elbow_coordinate_y = -5*sind(360 - angle_shoulder_deg);    
% end
% 
% end



