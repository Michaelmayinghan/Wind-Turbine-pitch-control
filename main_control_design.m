clear; close all; clc;
%% SYSTEM PARAMETERS
Lm  = 0.05;    % Motor Inductance
Rm  = 4;        % Armature Resistance
b   = 0.1;      % Viscous Friction
Kt  = 0.5;      % Torque Constant
Km  = 0.5;      % Back-EMF Constant
n   = 1/50;     % Gearing Factor

[J, umax] = pitch_robot(25057665);
fprintf('My (Adeel) specific parameters: n');
fprintf('J = %.6f kg*m^2\n', J);
fprintf('umax = %.4f V\n', umax);

%%  QUESTION 4.1: MATHEMATICAL MODELING (20 points) 
fprintf('\nQUESTION 4.1: Mathematical Modeling\n');

% Numerator coefficient
num_G = n * Kt;
% Denominator coefficients [s^3, s^2, s^1, s^0]
a3 = Lm * J;
a2 = Lm * b + Rm * J;
a1 = Rm * b + Km * Kt;
a0 = 0;  % integrator

den_G = [a3, a2, a1, a0];

% Create the transfer function
G = tf(num_G, den_G);

fprintf('Open-loop Transfer Function G(s):\n');
fprintf('G(s) = %.4f / (%.4f*s^3 + %.4f*s^2 + %.4f*s)\n', ...
    num_G, a3, a2, a1);

poles_G = pole(G);
zeros_G = zero(G);

fprintf('Poles of G(s):\n');
for i = 1:length(poles_G)
    fprintf('  p%d = %.6f + %.6fj\n', i, real(poles_G(i)), imag(poles_G(i)));
end
fprintf('\nZeros of G(s):\n');
if isempty(zeros_G)
    fprintf('  None\n');
else
    for i = 1:length(zeros_G)
        fprintf('  z%d = %.6f + %.6fj\n', i, real(zeros_G(i)), imag(zeros_G(i)));
    end
end

%MOVE ANALYSIS TO REPORT
% Check stability
is_stable = all(real(poles_G) < 0);
has_pole_at_origin = any(abs(poles_G) < 1e-10);

fprintf('\nStability Analysis\n');
if has_pole_at_origin
    fprintf('The system has a pole at s = 0 (integrator).\n');
    fprintf('Therefore, the open-loop system is NOT asymptotically stable.\n');
    fprintf('It is only marginally stable (bounded but not decaying response).\n');
    fprintf('(Lecture 7: A pole at the origin means the system is on the\n');
    fprintf(' stability boundary - not in the open LHP.)\n');
else
    if is_stable
        fprintf('All poles have negative real parts -> asymptotically stable.\n');
    else
        fprintf('The system has poles with non-negative real parts -> UNSTABLE.\n');
    end
end

% Plot pole-zero map
figure('Name', 'Q4.1 - Pole-Zero Map of G(s)');
pzmap(G);
title('Q4.1: Pole-Zero Map of Open-Loop G(s)');
grid on;
fprintf('\n');

%%  QUESTION 4.2: PROPORTIONAL CONTROL & STABILITY (25 points)
fprintf('QUESTION 4.2: Proportional Control & Stability\n');

% Search for optimal K 
K_test = linspace(0.1, 5000, 50000);
best_K = 0;
best_tr = inf;
best_OS = 0;
fprintf('Searching for optimal K where overshoot < 5%% with minimum rise time...\n');
for i = 1:length(K_test)
    K_i = K_test(i);
    Gc_i = feedback(K_i * G, 1);
    
    % Check if closed-loop is stable first
    cl_poles = pole(Gc_i);
    if any(real(cl_poles) > 0)
        break;  % if its already unstable theres no point going higher
    end
    
    try
        info = stepinfo(Gc_i);
        if info.Overshoot < 5 && info.RiseTime < best_tr
            best_K = K_i;
            best_tr = info.RiseTime;
            best_OS = info.Overshoot;
        end
    catch
        continue;
    end
end

K_opt = best_K;
fprintf('Optimal Proportional Gain: K = %.4f\n', K_opt);
fprintf('Resulting Overshoot:       %.4f%%\n', best_OS);
fprintf('Resulting Rise Time:       %.4f s\n\n', best_tr);

% Closed-loop system with optimal K
Gc_P = feedback(K_opt * G, 1);

% Plot step response
figure('Name', 'Q4.2 Step Response with P-Controller');
step(Gc_P);
title(sprintf('Q4.2: Closed-Loop Step Response (K = %.4f)', K_opt));
grid on;
info_P = stepinfo(Gc_P);

Go_P = K_opt * G;  % open-loop with proportional controller

% Bode Plot
figure('Name', 'Q4.2 Bode Plot');
margin(Go_P);
title(sprintf('Q4.2: Bode Plot of K*G(s) (K = %.4f)', K_opt));
grid on;

% Extract gain margin, phase margin, crossover frequencies
[Gm, Pm, Wcg, Wcp] = margin(Go_P);

fprintf('Open-Loop Frequency Response (K*G(s))\n');
fprintf('Gain Crossover Frequency (Wcp):   %.4f rad/s\n', Wcp);
fprintf('Phase Margin:                      %.4f degrees\n', Pm);
fprintf('Phase Crossover Frequency (Wcg):  %.4f rad/s\n', Wcg);
fprintf('Gain Margin:                       %.4f dB (= %.4f)\n', ...
    20*log10(Gm), Gm);
fprintf('\n');

% Bandwidth of closed-loop system
bw_cl = bandwidth(Gc_P);
fprintf('Closed-Loop Bandwidth:             %.4f rad/s\n\n', bw_cl);

% Nyquist Diagram
figure('Name', 'Q4.2 Nyquist Diagram');
nyquist(Go_P);
title(sprintf('Q4.2: Nyquist Diagram of K*G(s) (K = %.4f)', K_opt));
grid on;

K_critical = (a2 * a1) / (a3 * num_G);

fprintf('The system becomes unstable when K >= %.4f\n\n', K_critical);

% Method 2: Verify using Root Locus
figure('Name', 'Q4.2 Root Locus');
rlocus(G);
title('Q4.2: Root Locus of G(s)');
grid on;
hold on;
Gc_crit = feedback(K_critical * G, 1);
poles_crit = pole(Gc_crit);
plot(real(poles_crit), imag(poles_crit), 'rx', 'MarkerSize', 15, 'LineWidth', 2);
legend('Root Locus', sprintf('Poles at K_{crit} = %.2f', K_critical));
hold off;

% Verify critical gain with margin alone
fprintf('Verification: Gain margin of G(s) alone = %.4f (%.4f dB)\n', ...
    Gm, 20*log10(Gm));
fprintf('K_critical from Routh = %.4f\n', K_critical);
fprintf('K_opt * Gain Margin = %.4f * %.4f = %.4f \n', ...
    K_opt, Gm, K_opt * Gm);

% Store bandwidth for Q4.3 teammate reference
bw_P = bw_cl;
fprintf('=== VALUES FOR TEAMMATE (Q4.3) ===\n');
fprintf('P-controller bandwidth: %.4f rad/s\n', bw_P);
fprintf('Required Q4.3 bandwidth (4x): %.4f rad/s\n', 4 * bw_P);
fprintf('umax = %.4f V\n', umax);



%Question 4.3
disp('--- Q4.3 Results ---');
wc_req = 4 * 4.6202; 
PM_req = 65;
margin_safe = 12;

[mag_wc, phase_wc] = bode(G, wc_req); 
phi_add = PM_req - (180 + phase_wc) + margin_safe; 
phi_rad = phi_add * pi / 180; 

beta = (1 - sin(phi_rad)) / (1 + sin(phi_rad));
tau_D = 1 / (wc_req * sqrt(beta));
F_lead = tf([tau_D 1], [beta*tau_D 1]); 

K_p = 1 / (mag_wc * (1/sqrt(beta))); 

Kv = 20;
K_total_req = Kv / (num_G / a1); 

gamma = K_p / K_total_req; 
tau_I = 10 / wc_req;      
F_lag = tf([tau_I 1], [tau_I gamma]); 

C = K_p * F_lead * F_lag; 
L_new = C * G;        
T_new = feedback(L_new, 1); 

fprintf('Kp = %.4f, beta = %.4f, tau_D = %.4f\n', K_p, beta, tau_D);
fprintf('gamma = %.4f, tau_I = %.4f\n', gamma, tau_I);

[~, Pm_new, ~, Wcp_new] = margin(L_new);
fprintf('Actual wc = %.2f rad/s, PM = %.2f deg\n', Wcp_new, Pm_new);

info = stepinfo(T_new);
fprintf('Overshoot = %.2f%%, Rise time = %.4fs\n', info.Overshoot, info.RiseTime);

U_sys = feedback(C, G); 
[u, ~] = step(U_sys);
max_u = max(abs(u));
fprintf('Max u = %.2f V (Limit: %.2f V)\n\n', max_u, umax);

figure;
bode(K_opt * G, 'r--', L_new, 'b'); 
legend('P', 'Lead-Lag');
title('Bode Plot');
grid on;

figure;
step(Gc_P, 'r--', T_new, 'b');
legend('P', 'Lead-Lag');
title('Step Response');
grid on;

%Question 4.5
disp('--- Q4.5 Results ---');

A = [0 n 0; 
     0 -b/J Kt/J; 
     0 -Km/Lm -Rm/Lm];
B = [0; 0; 1/Lm];
C = [1 0 0]; 
D = 0;

z = 0.7; 


for wn = 1:5000

    p1 = -z*wn + j*wn*sqrt(1-z^2);
    p2 = -z*wn - j*wn*sqrt(1-z^2);
    p3 = -5 * z * wn; 
    
    K = place(A, B, [p1 p2 p3]);
    
    Acl = A - B*K;
    dc_gain = -C * inv(Acl) * B; 
    kr = 1/dc_gain;
    sys_u = ss(Acl, B*kr, -K, kr);
    [u, t] = step(sys_u);
    
    if max(abs(u)) > umax
        break 
    end
    
    best_K = K;
    best_kr = kr;
    best_wn = wn;
    sys_final = ss(Acl, B*kr, C, D);
    final_u = u;
    final_t = t;
end

fprintf('wn = %f\n', best_wn)
disp('K matrix:')
disp(best_K)
fprintf('kr = %f\n', best_kr)

info = stepinfo(sys_final);
fprintf('overshoot = %f %%\n', info.Overshoot)
fprintf('rise time = %f sec\n', info.RiseTime)
fprintf('max voltage = %f (limit is %f)\n', max(abs(final_u)), umax)

figure
plot(final_t, abs(final_u), 'b')
hold on

plot([0 max(final_t)], [umax umax], 'r--')
title('control signal u(t)')
xlabel('time')
ylabel('voltage')
grid on

figure
step(sys_final)
title('q4.5 step response')
grid on