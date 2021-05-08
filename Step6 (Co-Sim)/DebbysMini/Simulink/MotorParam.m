%=============================================%
% Initializes variables for the Simulink model%
%=============================================%

% input voltage
Vin = 5;

% Motor parameters
% Electrical
R  = 5.31;                  % Armature resistance       - ohms
L  = 0.227*10^-3;           % Armature inductance       - Henries
I_nl = 23.2*10^-3;          %No load Current            - Amps
w_nl = 10300*2*pi/60;       %No load speed              - rad/s

% OP AMP - Parameter determined from testfit.m script TF of [A1 A0]/[B2 B1 B0]
A1 = 0;
A0 = 7.452e05;
B2 = 0;
B1 = 1;
B0 = 3.134e05;

% Bridge
Km = 10.9*10^-3;            % Motor constant            - Nm/A
Kb = 10.95103*10^-3;        % Back-EMF constant         - V*s/rad

% Mechanical
Jm  = 0.966*10^-7;          % Motor inertia             - kgm^2
Jl  = 1.544425653*10^-7;    % Logo/lever arm inertia 	- kgm^2
J  = Jm+Jl;                 % Armature inertia          - kgm^2
B  = Km*I_nl/w_nl;          % Armature damping          - Nm*s/rad


% Display some results
G  = tf(Km, [L*J (L*B + R*J) R*B]);
H  = tf(Kb, 1);
OP = tf([A1 A0], [B2 B1 B0]);
TF1 = feedback(G,H)
TF = TF1*OP*(tf(1,[1 0]));
rlocus(TF);