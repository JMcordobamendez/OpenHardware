clear; close; clc;
Ts = 1e-5;
Tsc = 1e-3;
Tsco = 1e-3;
%Parámetros electricos motor
L = 2.04e-3; %2mH
R = 5.06; %Ohm
%Ensayos motor
I = (1e-3)*[73 76 80 81 87 91 94 97 100 102 106 108 108 115 125];
V = [1.7 2 2.2 2.4 3 3.5 4 4.5 5 5.5 6 6.5 7 8 9];
RPM = [44.5 52.6 60.4 64.8 85.4 104.5 121 136.7 155.4 171.6 189.6...
    205 225 259 291];
E = V - R.*I;
W = (2*pi/60)*RPM;
A = [W'];
Km = (A'*A)\(A'*E'); %V·s/rad
%Km = 0.23315;
f = @(x) Km*x;
figure;
hold on;
plot(W, E, '*r');
plot([0 W], f([0 W]), 'b');
grid();
xlabel('w(rad/s)');
ylabel('E(V)');
legend('Real', 'Regression');
title('E = Km·w');
%Parámetros mecánicos motor
A = [W'];
B = (A'*A)\(A'*(Km*I)');
f = @(x) B*x;
%B = 0.0022;
figure;
hold on;
plot(W, Km*I, '*r');
plot([0 W], f([0 W]), 'b');
grid();
xlabel('w(rad/s)');
ylabel('T(N·m)');
legend('Real', 'Regression');
title('T = B·w');
R_rueda = (6.66/2)*1e-2; %m 6.66cm
M = 2.5; %Kg
J = M*R_rueda^2; %Kg·m2
%Fuente conmutada alimenta
I_feed = 0.24; %240 mA
Fsw = 300e3; %300 KHz
DI_feed = 0.4*I_feed;
DV_feed = 0.01*5;
L_feed = 5*(1-5/12.8)/(DI_feed*Fsw);
C_feed=DI_feed/(8*DV_feed*Fsw);
B_sat = 0.3; %T
uo = 4*pi*1e-7;
n_min = 30;
Ae_est = (L_feed*(I_feed+DI_feed/2 + 0.8))/(B_sat*n_min)*(100/1)^2; %cm2
Ae = 19.7*(1/1000)^2; %m2
le = 38.5/1000; %m
AL = 5470e-9; %L =AL·n2
%n_neces = round(sqrt(L_feed*le/(u*uo*Ae)));
n_neces = round(sqrt(L_feed/AL));
n_min_neces = (L_feed*(I_feed+DI_feed/2))/(B_sat*Ae);
%Perdidas diodo (SK52C)
I_Diodo_avg = (1-5/12.8)*I_feed*1.2;
I_Diodo_rms = sqrt(1-5/12.8)*I_feed*1.2;
V_reverse = 5; %V
VF_Diodo = 0.2; %V
R_Diodo = (0.5-VF_Diodo)/5; %Ohm
C_Diodo = 500e-12; %500 pF
P_conm = 1/2*C_Diodo*Fsw*V_reverse^2;
P_cond = VF_Diodo*I_Diodo_avg + R_Diodo*I_Diodo_rms^2;
P_Diodo = P_conm + P_cond;
Rth = 50; %ºC/W
Temp_Diodo = 40+Rth*P_Diodo;
%Estudio frecuencia conmutación motor
Fswm = 30e3; %Hz
N_nom = 400; %rpm
E_nom = N_nom*2*pi/60*Km; %fcem V
ILM_Nom = B*N_nom*2*pi/Km/60; %300 mA a 300 rpm
dILM = (E_nom*(1-E_nom/12.8))/(L*Fswm*ILM_Nom)*100;
%Modelo espacio de estados ampliado
A_mat = [-R/L -Km/L 0;Km/J -B/J 0;0 -1 0];
B_mat = [1/L 0 0]';
%Q = [1 0 0;0 1 0;0 0 100]; %K = [0.1433 1.1577 -10] inestable bajas
%velocidades
Q = [1 0 0;0 1 0;0 0 10];
R_mat = 1;
K = lqr(A_mat,B_mat,Q,R_mat);
figure;
step(A_mat-B_mat*K,[0 0 30]',[1 0 0;0 1 0;0 0 1],[0 0 0]');
grid();
%Sensor corriente
m = 1/254.71; %A/bits
%% Observador de estado
Ao = [-R/L -Km/L; Km/J -B/J]';
Co = [1 0; 0 1]';
Qo = [1 0;0 10000];
Ro = [1 0;0 1];
Ke = lqr(Ao,Co,Qo,Ro);
poles_t = eig(Ao-Co*Ke);
Ke = Ke';
disp(max(poles_t));




