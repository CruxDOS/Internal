%% Pre-simulation setup
% This file is included in modelsim.m 

% Inertia matrix
phyparam.I = [	phyparam.I_xx	0		0		;
		0		phyparam.I_yy	0		;
		0		0		phyparam.I_zz	];

params = struct( 			...
	'd_t', 	{time.delta},		...
	'm', 	{phyparam.m},   	...
	'g', 	{phyparam.g},   	...
	'k', 	{phyparam.k},   	...
	'L', 	{phyparam.L},   	...
	'b', 	{phyparam.b},   	...
	'I_xx',	{phyparam.I_xx},	...
	'I_yy',	{phyparam.I_yy},	...
	'I_zz',	{phyparam.I_zz} 	...
	);

%% Discrete time sequence
time.seq = time.start:time.delta:time.end;
time.N = numel(time.seq);

%% Randomize initial distrubance
d_theta_init = deg2rad(d_theta_devi.*ones(3,1).*(2*rand(3,1) - 1)); % apply disturbance at beginning

state = struct(			...
	'theta',	{0},	...
	'prev_d_theta',	{0}	...
	);
%state.prev_d_theta = d_theta_init; % assuming previous d_theta has the same value

PD_param = struct(	...
	'Kd',	{Kd},	...
	'Kp',	{Kp}	...
	);

m1pos = [phyparam.L*cos(m1ang); phyparam.L*sin(m1ang); 0]; % convert to rectangular coordinates
m2pos = [phyparam.L*cos(m2ang); phyparam.L*sin(m2ang); 0];
m3pos = [phyparam.L*cos(m3ang); phyparam.L*sin(m3ang); 0];
m4pos = [phyparam.L*cos(m4ang); phyparam.L*sin(m4ang); 0];

% Matrices to save data
X_data = zeros(3, time.N);
M1x_data = zeros(3, time.N);
M2x_data = zeros(3, time.N);
M3x_data = zeros(3, time.N);
M4x_data = zeros(3, time.N);
F1x_data = zeros(3, time.N);
F2x_data = zeros(3, time.N);
F3x_data = zeros(3, time.N);
F4x_data = zeros(3, time.N);
Fav_data = zeros(3, time.N);
Theta_data = zeros(3, time.N);
D_theta_data = zeros(3, time.N);
V_data = zeros(3, time.N);
R_data = zeros(4, time.N);
A_data = zeros(3, time.N);

