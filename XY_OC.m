clear all
clc

%% Trj Params
T_init = 0.;
T_final = 3.;
dt = 0.01;

N = T_final/dt;

%% Initial state
x0 = 0.;
dx0 = 0.;
ddx0 = 0.;

y0 = 0.;
dy0 = 0.;
ddy0 = 0.;

%% Final state
xf = -1.;
dxf = 0.;
ddxf = 0.;

yf = 2.0;
dyf = 0.;
ddyf = 0.;

%% Collocated states
x200 = .5;

%% Variables
x = zeros(N+1, 1);   %position x
dx = zeros(N+1, 1);  %velocity x
ddx = zeros(N+1, 1); %acceleration x

y = zeros(N+1, 1);   %position y
dy = zeros(N+1, 1);  %velocity y
ddy = zeros(N+1, 1); %acceleration y
%% Ctrl input
ux = zeros(N, 1);   %jerk x
uy = zeros(N, 1);   %jerk y

%% Tot Variables
X = [ddx; ddy; dx; dy; x; y; ux; uy]; %[6*(N+1) + 2*N x 1]

%% Acceleration constr --> A*X = Iv*ddx0
Iv_acc_x = ones(N+1,1)*ddx0;
Iv_acc_y = ones(N+1,1)*ddy0;
Iv_acc = [Iv_acc_x; Iv_acc_y];

A_acc = [eye(2*(N+1)) zeros(2*(N+1), 4*(N+1))];
B_acc_x = zeros(1, 2*N);
B_acc_y = zeros(1, 2*N);
for i = 1:1:N
    B_acc_x = [B_acc_x; ones(1,i) zeros(1, N-i) zeros(1,N)];
    B_acc_y = [B_acc_y; zeros(1,N) ones(1,i) zeros(1, N-i)];
end
B_acc = [B_acc_x; B_acc_y]; 
A_acc = [A_acc -B_acc*dt];

A_acc_f_x = [zeros(1,N) 1 zeros(1, 5*(N+1)) zeros(1, 2*N)];
b_acc_f_x = ddxf;
A_acc_f_y = [zeros(1,N+1) zeros(1,N) 1 zeros(1, 4*(N+1)) zeros(1, 2*N)];
b_acc_f_y = ddyf;
A_acc_f = [A_acc_f_x; A_acc_f_y];
b_acc_f = [b_acc_f_x; b_acc_f_y];
%% Velocity constr
Iv_vel_x = ones(N+1,1)*dx0;
Iv_vel_y = ones(N+1,1)*dy0;
Iv_vel = [Iv_vel_x; Iv_vel_y];

A_vel = [eye(2*(N+1)) zeros(2*(N+1), 2*(N+1)) zeros(2*(N+1), 2*N)];
B_vel_x = [];
B_vel_y = [];
for i = 1:1:N+1
    B_vel_x = [B_vel_x; ones(1,i) zeros(1, N+1-i) zeros(1,N+1)];
    B_vel_y = [B_vel_y; zeros(1, N+1) ones(1,i) zeros(1, N+1-i)];
end
B_vel = [B_vel_x; B_vel_y];
A_vel = [-B_vel*dt A_vel];

A_vel_f_x = [zeros(1,2*(N+1)) zeros(1, N) 1 zeros(1,3*(N+1)) zeros(1, 2*N)];
b_vel_f_x = dxf;

A_vel_f_y = [zeros(1,2*(N+1)) zeros(1, N+1) zeros(1, N) 1 zeros(1,2*(N+1)) zeros(1, 2*N)];
b_vel_f_y = dyf;
A_vel_f = [A_vel_f_x; A_vel_f_y];
b_vel_f = [b_vel_f_x; b_vel_f_y];
%% Position constr
Iv_pos_x = ones(N+1,1)*x0;
Iv_pos_y = ones(N+1,1)*y0;
Iv_pos = [Iv_pos_x; Iv_pos_y];

A_pos = [eye(2*(N+1)) zeros(2*(N+1), 2*N)];
B_pos_x = [];
B_pos_y = [];
for i = 1:1:N+1
    B_pos_x = [B_pos_x; ones(1,i) zeros(1, N+1-i) zeros(1, N+1)];
    B_pos_y = [B_pos_y; zeros(1, N+1) ones(1,i) zeros(1, N+1-i)];
end
B_pos = [B_pos_x; B_pos_y];
A_pos = [-(1/2)*B_pos*dt^2 -B_pos*dt A_pos];

A_pos_f_x = [zeros(1,4*(N+1)) zeros(1, N) 1 zeros(1, (N+1)) zeros(1, 2*N)];
b_pos_f_x = xf;
A_pos_200_x = [zeros(1,4*(N+1)) zeros(1, 199) 1 zeros(1,(N+1)-200) zeros(1, (N+1)) zeros(1, 2*N)];
b_pos_200_x = x200;

A_pos_f_y = [zeros(1,5*(N+1)) zeros(1, N) 1 zeros(1, 2*N)];
b_pos_f_y = yf;
A_pos_200_y = [zeros(1,6*(N+1)) zeros(1, 2*N)];
b_pos_200_y = 0;
A_pos_f = [A_pos_f_x; A_pos_f_y];
b_pos_f = [b_pos_f_x; b_pos_f_y];
A_pos_200 = [A_pos_200_x; A_pos_200_y];
b_pos_200 = [b_pos_200_x; b_pos_200_y];
%% Cost function
H = eye(6*(N+1)+2*N);
f = [];

%% Constraints
Aeq = [A_acc; A_acc_f; A_vel; A_vel_f; A_pos; A_pos_200; A_pos_f];
beq = [Iv_acc; b_acc_f; Iv_vel; b_vel_f; Iv_pos; b_pos_200; b_pos_f];

%% QUADPROG
X = quadprog(H,f,[],[],Aeq,beq);

figure();
plot(X( (4*(N+1)+1):1:(5*(N+1)) , 1), X( (5*(N+1)+1):1:(6*(N+1)) , 1)); title('x-y');
hold on;
plot( X(4*(N+1)+200,1) , X(5*(N+1)+200,1),'o'); hold on;
plot(x0,y0,'og'); hold on; plot(xf,yf,'or');  axis([-2 2 -2 2]);
figure();
subplot(3,2,1); plot(X( 1:1:(N+1) , 1)); title('ddx'); subplot(3,2,2); plot(X( (N+2):1:2*(N+1) , 1)); title('ddy');
subplot(3,2,3); plot(X( 2*(N+1)+1:1:3*(N+1) , 1)); title('dx'); subplot(3,2,4); plot(X( 3*(N+1)+1:1:4*(N+1) , 1)); title('dy');
subplot(3,2,5); plot(X( 6*(N+1)+1:1:6*(N+1)+N , 1)); title('ux'); subplot(3,2,6); plot(X( 6*(N+1)+1+N+1:1:6*(N+1)+2*N , 1)); title('uy');