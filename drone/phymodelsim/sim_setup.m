%% Pre-simulation setup
% This file is included in modelsim.m 

% Inertia matrix
phyparam.I = [	phyparam.I_xx	0		0		;
		0		phyparam.I_yy	0		;
		0		0		phyparam.I_zz	];
%% Discrete time sequence
time.seq = time.start:time.delta:time.end;
time.N   = numel(time.seq);
time.t   = 0;
time.tc  = 0;

%% Display parameters
dispparam.time_seq = time.start:dispparam.time_delta:time.end;
dispparam.time_N   = numel(dispparam.time_seq);
dispparam.tc       = 0;

%% Randomize initial distrubance
if theta_rand == 1
	d_theta_init = deg2rad(d_theta_devi_rand.*ones(3,1).*(2*rand(3,1) - 1)); % apply disturbance at beginning
else
	d_theta_init = deg2rad(d_theta_devi_init);
end

disp('Initial diviation in angular velocity, in degrees/second:');
disp(['	x: ', num2str(rad2deg(d_theta_init(1)))]);
disp(['	y: ', num2str(rad2deg(d_theta_init(2)))]);
disp(['	z: ', num2str(rad2deg(d_theta_init(3)))]);
disp('');

%% Controller parameters and output queue
ctrlstate.theta        = 0;
ctrlstate.prev_d_theta = 0;
ctrlstate.out_q        = [];

%% Initial thrust
rotate = ones(4,1) * phyparam.m*phyparam.g/(4*phyparam.k*(cos(state.theta(1))*cos(state.theta(2))));

%% Motor positions
phyparam.m1pos = [phyparam.L*cos(phyparam.m1ang); phyparam.L*sin(phyparam.m1ang); 0]; % convert to rectangular coordinates
phyparam.m2pos = [phyparam.L*cos(phyparam.m2ang); phyparam.L*sin(phyparam.m2ang); 0];
phyparam.m3pos = [phyparam.L*cos(phyparam.m3ang); phyparam.L*sin(phyparam.m3ang); 0];
phyparam.m4pos = [phyparam.L*cos(phyparam.m4ang); phyparam.L*sin(phyparam.m4ang); 0];

% Matrix of structs to save data
datatype.X       = zeros(3, 1);
datatype.M1x     = zeros(3, 1);
datatype.M2x     = zeros(3, 1);
datatype.M3x     = zeros(3, 1);
datatype.M4x     = zeros(3, 1);
datatype.F1x     = zeros(3, 1);
datatype.F2x     = zeros(3, 1);
datatype.F3x     = zeros(3, 1);
datatype.F4x     = zeros(3, 1);
datatype.Fa1x    = zeros(3, 1);
datatype.Fa2x    = zeros(3, 1);
datatype.Fa3x    = zeros(3, 1);
datatype.Fa4x    = zeros(3, 1);
datatype.Rotate  = zeros(4, 1);
datatype.A       = zeros(3, 1);
datatype.Theta   = zeros(3, 1);
datatype.D_theta = zeros(3, 1);
datatype.V       = zeros(3, 1);

DATA = repmat (datatype, 1, dispparam.time_N);
