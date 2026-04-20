clc; clear all; close all;

format long g

%% read data
% base body
A0 = euler_zxz(0,0,0);
r0 = zeros(3,1);
w0 = zeros(3,1);
dr0 = zeros(3,1);
w0t = skew(w0);
dY0h = zeros(6,1);
Y0h = zeros(6,1);

% body1
q1 = 0;
dq1 = 0;
s01p = [0;0;0];
C01 = euler_zxz(0, pi, 0);
rho1p = [0.0026336; -1.68446e-05; -0.111117];
C11 = euler_zxz(0, 0, 0);
m1 = 10.0123496865811;
Ixx = 8.29601614566715e-002; Ixy = -3.28089653994623e-004;
Iyy = 3.91199810914461e-002; Iyz = 2.09012210735718e-006;
Izz = 6.49458588226152e-002; Izx = -1.19428008908532e-004;
J1p = [Ixx, Ixy, Izx; 
        Ixy, Iyy, Iyz; 
        Izx, Iyz, Izz];

% body2
q2 = 0;
dq2 = 0;
s12p = [0;0;-0.22];
C12 = euler_zxz(0, pi/2, 0);
rho2p = [6.90559e-05;-0.0851548;-0.00686211];
C22 = euler_zxz(pi, pi/2, pi);
m2 = 10.4391437674567;
Ixx = 7.80970193464117e-002; Ixy = 3.53131298101708e-005;
Iyy = 2.55871855807303e-002; Iyz = 5.47282289489732e-003;
Izz = 7.45746466344453e-002; Izx = -8.81620777833472e-006;
J2p = [Ixx, Ixy, Izx;
        Ixy, Iyy, Iyz;
        Izx, Iyz, Izz];

% body3
q3 = 0;
dq3 = 0;
s23p = [0;-0.23;0];
C23 = euler_zxz(-pi/2, 0, 0);
rho3p = [0.0969069;-3.20036e-05;-0.00548574];
C33 = euler_zxz(-pi/2, pi/2, pi);
m3 = 10.3406497234359;
Ixx = 7.92872354716476e-002; Ixy = 8.26042768438978e-006;
Iyy = 2.27404242493157e-002; Iyz = 4.31986606376804e-003;
Izz = 7.81409250910383e-002; Izx = 5.43833216931251e-006;
J3p = [Ixx, Ixy, Izx;
        Ixy, Iyy, Iyz;
        Izx, Iyz, Izz];

% body4
q4 = 0;
dq4 = 0;
s34p = [0.23;0;0];
C34 = euler_zxz(0, pi, 0);
rho4p = [0.0675884;0.00443192;0.000679202];
C44 = euler_zxz(-pi/2, pi/2, 0);
m4 = 7.01416597186014;
Ixx = 3.93493190184971e-002; Ixy = -9.26169916890393e-004;
Iyy = 1.11764686166838e-002; Iyz = 2.04330936352529e-004;
Izz = 4.10218257620852e-002; Izx = -8.43406506016272e-006;
J4p = [Ixx, Ixy, Izx;
        Ixy, Iyy, Iyz;
        Izx, Iyz, Izz];

% end effector
s4ep = [0.18;0;0];
C4e = euler_zxz(-pi/2, 0, 0);

%% read data
rec_data_raw = readtable('../recurdyn/rec_data_free_fall.csv');
rec_data = rec_data_raw{:, 2:end};

%% system parameters
t_c = 0;
h = 0.001;
g = -9.80665;
u_vec = [0;0;1];

q1 = rec_data(1, 32);
q2 = rec_data(1, 33);
q3 = rec_data(1, 34);
q4 = rec_data(1, 35);
dq1 = rec_data(1, 36);
dq2 = rec_data(1, 37);
dq3 = rec_data(1, 38);
dq4 = rec_data(1, 39);

%% position and orientation of each body
A01pp = [cos(q1), -sin(q1), 0; sin(q1), cos(q1), 0; 0, 0, 1];
A12pp = [cos(q2), -sin(q2), 0; sin(q2), cos(q2), 0; 0, 0, 1];
A23pp = [cos(q3), -sin(q3), 0; sin(q3), cos(q3), 0; 0, 0, 1];
A34pp = [cos(q4), -sin(q4), 0; sin(q4), cos(q4), 0; 0, 0, 1];

A1 = A0*C01*A01pp;
A2 = A1*C12*A12pp;
A3 = A2*C23*A23pp;
A4 = A3*C34*A34pp;

s01 = A0*s01p;
s12 = A1*s12p;
s23 = A2*s23p;
s34 = A3*s34p;

r1 = r0 + s01;
r2 = r1 + s12;
r3 = r2 + s23;
r4 = r3 + s34;

rho1 = A1*rho1p;
rho2 = A2*rho2p;
rho3 = A3*rho3p;
rho4 = A4*rho4p;

r1c = r1 + rho1;
r2c = r2 + rho2;
r3c = r3 + rho3;
r4c = r4 + rho4;

s4e = A4*s4ep;
re = r4 + s4e;
Ae = A4*C4e;
rpy = mat2rpy(Ae);

%% velocity of each body
H1 = A0*C01*u_vec;
H2 = A1*C12*u_vec;
H3 = A2*C23*u_vec;
H4 = A3*C34*u_vec;

w1 = w0 + H1*dq1;
w2 = w1 + H2*dq2;
w3 = w2 + H3*dq3;
w4 = w3 + H4*dq4;

w1t = skew(w1);
w2t = skew(w2);
w3t = skew(w3);
w4t = skew(w4);

dr1 = dr0 + w0t*s01;
dr2 = dr1 + w1t*s12;
dr3 = dr2 + w2t*s23;
dr4 = dr3 + w3t*s34;

dre = dr4 + w4t*s4e;

r1t = skew(r1);
r2t = skew(r2);
r3t = skew(r3);
r4t = skew(r4);

B1 = [r1t*H1; H1];
B2 = [r2t*H2; H2];
B3 = [r3t*H3; H3];
B4 = [r4t*H4; H4];

dr1t = skew(dr1);
dr2t = skew(dr2);
dr3t = skew(dr3);
dr4t = skew(dr4);

dr1c = dr1 + w1t*rho1;
dr2c = dr2 + w2t*rho2;
dr3c = dr3 + w3t*rho3;
dr4c = dr4 + w4t*rho4;

dH1 = w0t*H1;
dH2 = w1t*H2;
dH3 = w2t*H3;
dH4 = w3t*H4;

D1 = [dr1t*H1 + r1t*dH1; dH1]*dq1;
D2 = [dr2t*H2 + r2t*dH2; dH2]*dq2;
D3 = [dr3t*H3 + r3t*dH3; dH3]*dq3;
D4 = [dr4t*H4 + r4t*dH4; dH4]*dq4;

Y1h = Y0h + B1*dq1;
Y2h = Y1h + B2*dq2;
Y3h = Y2h + B3*dq3;
Y4h = Y3h + B4*dq4;

%% mass matrix & force vector
A1_C11 = A1*C11;
A2_C22 = A2*C22;
A3_C33 = A3*C33;
A4_C44 = A4*C44;

J1c = A1_C11*J1p*A1_C11';
J2c = A2_C22*J2p*A2_C22';
J3c = A3_C33*J3p*A3_C33';
J4c = A4_C44*J4p*A4_C44';

r1ct = skew(r1c);
r2ct = skew(r2c);
r3ct = skew(r3c);
r4ct = skew(r4c);

dr1ct = skew(dr1c);
dr2ct = skew(dr2c);
dr3ct = skew(dr3c);
dr4ct = skew(dr4c);

f1c = [0;0;m1*g];
f2c = [0;0;m2*g];
f3c = [0;0;m3*g];
f4c = [0;0;m4*g];

t1c = [0;0;0];
t2c = [0;0;0];
t3c = [0;0;0];
t4c = [0;0;0];

M1h = [m1*eye(3), -m1*r1ct; m1*r1ct, J1c - m1*r1ct*r1ct];
M2h = [m2*eye(3), -m2*r2ct; m2*r2ct, J2c - m2*r2ct*r2ct];
M3h = [m3*eye(3), -m3*r3ct; m3*r3ct, J3c - m3*r3ct*r3ct];
M4h = [m4*eye(3), -m4*r4ct; m4*r4ct, J4c - m4*r4ct*r4ct];

Q1h = [f1c + m1*dr1ct*w1; t1c + r1ct*f1c + m1*r1ct*dr1ct*w1 - w1t*J1c*w1];
Q2h = [f2c + m2*dr2ct*w2; t2c + r2ct*f2c + m2*r2ct*dr2ct*w2 - w2t*J2c*w2];
Q3h = [f3c + m3*dr3ct*w3; t3c + r3ct*f3c + m3*r3ct*dr3ct*w3 - w3t*J3c*w3];
Q4h = [f4c + m4*dr4ct*w4; t4c + r4ct*f4c + m4*r4ct*dr4ct*w4 - w4t*J4c*w4];

%% equations of motion
K4 = M4h;
K3 = M3h + K4;
K2 = M2h + K3;
K1 = M1h + K2;

L4 = Q4h;
L3 = Q3h + L4 - K4*D4;
L2 = Q2h + L3 - K3*D3;
L1 = Q1h + L2 - K2*D2;

M = [B1'*K1*B1, B1'*K2*B2, B1'*K3*B3, B1'*K4*B4;
     B2'*K2*B1, B2'*K2*B2, B2'*K3*B3, B2'*K4*B4;
     B3'*K3*B1, B3'*K3*B2, B3'*K3*B3, B3'*K4*B4;
     B4'*K4*B1, B4'*K4*B2, B4'*K4*B3, B4'*K4*B4];

Q = [B1'*(L1 - K1*D1);
     B2'*(L2 - K2*(D1 + D2));
     B3'*(L3 - K3*(D1 + D2 + D3));
     B4'*(L4 - K4*(D1 + D2 + D3 + D4))];

ddq = M\Q;

fprintf('mat q : %3.7f, %3.7f, %3.7f, %3.7f\n', q1, q2, q3, q4);
fprintf('rec q : %3.7f, %3.7f, %3.7f, %3.7f\n', rec_data(1, 32:35));
fprintf('mat dq: %3.7f, %3.7f, %3.7f, %3.7f\n', dq1, dq2, dq3, dq4);
fprintf('rec dq: %3.7f, %3.7f, %3.7f, %3.7f\n', rec_data(1, 36:39));
fprintf('mat re: %3.7f, %3.7f, %3.7f, %3.7f, %3.7f, %3.7f\n', re(1), re(2), re(3), rpy(1), rpy(2), rpy(3));
fprintf('rec re: %3.7f, %3.7f, %3.7f, %3.7f, %3.7f, %3.7f\n', rec_data(1, 2:7));
% fprintf('mat dre: %3.7f, %3.7f, %3.7f, %3.7f, %3.7f, %3.7f\n', dre(1), dre(2), dre(3), w4(1), w4(2), w4(3));
% fprintf('rec dre: %3.7f, %3.7f, %3.7f, %3.7f, %3.7f, %3.7f\n', rec_data(1, 8:13));
fprintf('mat ddq: %3.7f, %3.7f, %3.7f, %3.7f\n', ddq(1), ddq(2), ddq(3), ddq(4));
fprintf('rec ddq: %3.7f, %3.7f, %3.7f, %3.7f\n', rec_data(1, 40:43));
