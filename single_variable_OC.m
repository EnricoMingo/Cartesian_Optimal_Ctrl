clear all
clc

%% Trj Params
T_init = 0.;
T_final = 4.;
dt = 0.01;

N = T_final/dt;

%% Initial state
x0 = 0.;
dx0 = 0.;
ddx0 = 0.;

%% Final state
xf = 0.4;
dxf = 0.;
ddxf = 0.;

%% Collocated states
x200 = 0.2;

%% Variables
x = zeros(N+1, 1);   %position x
dx = zeros(N+1, 1);  %velocity x
ddx = zeros(N+1, 1); %acceleration x

%% Ctrl input
u = zeros(N, 1);   %jerk x

%% Tot Variables
X = [ddx; dx; x; u]; %[3*(N+1) + N x 1]

%% Acceleration constr --> A*X = Iv*ddx0
Iv_acc = ones(N+1,1)*ddx0;
A_acc = [eye(N+1) zeros(N+1, 2*(N+1))];
B_acc = zeros(1, N);
for i = 1:1:N
    B_acc = [B_acc; ones(1,i) zeros(1, N-i)];
end
A_acc = [A_acc -B_acc*dt];
A_acc_f = [zeros(1,N) 1 zeros(1, 2*(N+1)) zeros(1, N)];
b_acc_f = ddxf;

%% Velocity constr
Iv_vel = ones(N+1,1)*dx0;
A_vel = [eye(N+1) zeros(N+1, N+1) zeros(N+1, N)];
B_vel = [];
for i = 1:1:N+1
    B_vel = [B_vel; ones(1,i) zeros(1, N+1-i)];
end
A_vel = [-B_vel*dt A_vel];
A_vel_f = [zeros(1,N+1) zeros(1, N) 1 zeros(1,N+1) zeros(1, N)];
b_vel_f = dxf;

%% Position constr
Iv_pos = ones(N+1,1)*x0;
A_pos = [eye(N+1) zeros(N+1, N)];
B_pos = [];
for i = 1:1:N+1
    B_pos = [B_pos; ones(1,i) zeros(1, N+1-i)];
end
A_pos = [-(1/2)*B_pos*dt^2 -B_pos*dt A_pos];
A_pos_f = [zeros(1,2*(N+1)) zeros(1, N) 1 zeros(1, N)];
b_pos_f = xf;
A_pos_200 = [zeros(1,2*(N+1)) zeros(1, 199) 1 zeros(1,(N+1)-200) zeros(1, N)];
b_pos_200 = x200;


%% Cost function
H = eye(3*(N+1)+N);
f = [];

%% Constraints
Aeq = [A_acc; A_acc_f; A_vel; A_vel_f; A_pos; A_pos_200; A_pos_f];
beq = [Iv_acc; b_acc_f; Iv_vel; b_vel_f; Iv_pos; b_pos_200; b_pos_f];

%% QUADPROG
X = quadprog(H,f,[],[],Aeq,beq);


subplot(2,2,1); plot(X( (2*(N+1)+1):1:(3*(N+1)) , 1)); title('x');
subplot(2,2,2); plot(X( ((N+1)+1):1:(2*(N+1)) , 1)); title('dx');
subplot(2,2,3); plot(X( 1:1:(N+1) , 1)); title('ddx');
subplot(2,2,4): plot(X( (3*(N+1)+1):1:(3*(N+1)+N) , 1)); title('u');
