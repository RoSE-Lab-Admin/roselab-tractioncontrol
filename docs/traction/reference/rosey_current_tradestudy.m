%% RoSEy Trade Study: per-side (summed) vs per-wheel motor current for slip estimation
% Question: the slip estimator inverts W(theta_f, s) and T(theta_f, s) per wheel.
% If only the summed current of one side (front + rear motor) is used, how much
% slip error does that add, compared with using each motor's own current?
%
% Uses the validated model (rosey_validation.m, variant B: theta_f' term,
% lambda_s = 0.5) and the bench data in "Motor Characterization.xlsx".
%
% Assumptions (straight-line driving, the project scope):
%   - Both wheels on a side turn at the same speed and the chassis is rigid,
%     so front and rear wheels share the same slip s.
%   - Front/rear load split is unknown to a per-side estimator (it uses W_avg).
% Author: Ayooluwa Adeyinka, RoSE Lab, Colorado School of Mines

1;

%% ======================= FUNCTIONS (same model as main script) ===========
function sig = normalStress(th, thf, s, P)
    N = P.n0 + P.n1*s;  thr = P.c3*thf;  thm = (P.c1 + P.c2*s)*thf;
    kEq = (P.kc/P.b + P.kphi)*1e3*P.r^N;  sig = zeros(size(th));
    front = th >= thm;
    sig(front) = kEq*max(cos(th(front)) - cos(thf), 0).^N;
    rear = ~front;  thEq = thf - (th(rear) - thr)./(thm - thr).*(thf - thm);
    sig(rear) = kEq*max(cos(thEq) - cos(thf), 0).^N;
end
function tau = shearStress(th, thf, s, sig, P)
    th1 = acos(min((P.r - P.r*(1 - cos(thf)))/P.Rj, 1));
    j   = P.rs*((th1 - th) - (1 - s).*(sin(th1) - sin(th)));
    tau = (P.c + sig.*tan(P.phi)).*(1 - exp(-j./P.K));
end
function [W, DP, T] = wheelForces(thf, s, P)
    th = linspace(P.c3*thf, thf, P.nInt + 1);
    sig = normalStress(th, thf, s, P);  tau = shearStress(th, thf, s, sig, P);
    W  = P.b*simpson(P.r*sig.*cos(th) + P.rs*tau.*sin(th), th);
    DP = P.b*simpson(P.rs*tau.*cos(th) - P.r*sig.*sin(th), th);
    T  = P.rs^2*P.b*simpson(tau, th);
end
function I = simpson(f, x)
    n = numel(x) - 1;  h = (x(end) - x(1))/n;
    I = h/3*(f(1) + 4*sum(f(2:2:n)) + 2*sum(f(3:2:n-1)) + f(end));
end
function thf = solveEntryAngle(Wt, s, P)
    lo = 1e-4;  hi = P.thfMax;
    if wheelForces(hi, s, P) < Wt, thf = NaN; return; end
    for it = 1:60
        mid = 0.5*(lo + hi);
        if wheelForces(mid, s, P) > Wt, hi = mid; else, lo = mid; end
    end
    thf = 0.5*(lo + hi);
end
function T = torqueAt(W, s, P)
    [~, ~, T] = wheelForces(solveEntryAngle(W, s, P), s, P);
end
function s = slipFromTorque(W, Tm, P)
% Invert T(s) at known load W by bisection (T is monotonic in s here)
    lo = 0;  hi = 0.9;
    for it = 1:40
        mid = 0.5*(lo + hi);
        if torqueAt(W, mid, P) > Tm, hi = mid; else, lo = mid; end
    end
    s = 0.5*(lo + hi);
end

%% ======================= MAIN SCRIPT =====================================
clc; close all;

%% 1. RoSEy wheel + soil (same as rosey_terramechanics_octave.m) ---------
P.r = 0.1515;  P.b = 0.05158;  P.h = 0.013;  P.lam_s = 0.5;
P.rs = P.r + P.lam_s*P.h;  P.Rj = P.r + P.h;
P.kc = 15.6;  P.kphi = 2407.4;  P.n0 = 0.81;  P.n1 = 1.16;
P.c = 250;  P.phi = deg2rad(31.9);  P.K = 0.011;
P.c1 = 0.5;  P.c2 = -0.3;  P.c3 = 0;  P.nInt = 60;  P.thfMax = deg2rad(80);
Wavg = 16.01*9.81/4;                         % static load per wheel [N]

%% 2. Current -> torque calibration from bench data ----------------------
% Master sheet of "Motor Characterization.xlsx": mean current [mA] at applied
% output torque [kg-cm]. Columns: torque, current.
kgcm = 0.0980665;                            % N*m per kg-cm
bench = { [0 250.56; 52.10 2781.74; 72.888 4759.00], ...   % Motor 1
          [0 253.79; 52.10 2810.09; 77.596 4404.67], ...   % Motor 2
          [0 258.47; 51.08 2534.28; 71.604 4047.60], ...   % Motor 3
          [0 263.14; 52.40 3014.84; 68.736 4206.91] };     % Motor 4
fprintf('Current -> output torque fit, T = Kt*(I - I0)\n');
Kt = zeros(1,4);  I0 = Kt;
for m = 1:4
    T_Nm = bench{m}(:,1)*kgcm;  I_A = bench{m}(:,2)/1000;
    p = polyfit(I_A, T_Nm, 1);               % T = p1*I + p2
    Kt(m) = p(1);  I0(m) = -p(2)/p(1);
    fprintf('  Motor %d: Kt = %.2f N*m/A, I0 = %.0f mA\n', m, Kt(m), I0(m)*1000);
end
fprintf('  Spread in Kt across motors: %.0f%% (max/min - 1)\n\n', 100*(max(Kt)/min(Kt) - 1));
KtMean = mean(Kt);

%% 3. Error from unknown front/rear load split ---------------------------
% True: front wheel carries 2*Wavg*f, rear 2*Wavg*(1-f), both at slip sTrue.
% Per-side estimator only knows T_side = T_f + T_r and assumes W_avg each.
fFront = [0.50 0.45 0.40 0.35 0.30];
sTrue  = [0.10 0.20 0.30];
fprintf('Slip error from summing a side (unknown load split)\n');
fprintf('%10s', 'front %');  fprintf('   s=%.2f', sTrue);  fprintf('\n');
errSplit = zeros(numel(fFront), numel(sTrue));
for i = 1:numel(fFront)
    fprintf('%10.0f', 100*fFront(i));
    for k = 1:numel(sTrue)
        Tside = torqueAt(2*Wavg*fFront(i), sTrue(k), P) + torqueAt(2*Wavg*(1 - fFront(i)), sTrue(k), P);
        sHat  = slipFromTorque(Wavg, Tside/2, P);
        errSplit(i,k) = sHat - sTrue(k);
        fprintf('   %+.3f', errSplit(i,k));
    end
    fprintf('\n');
end

%% 4. Sensitivity and noise: how much slip does one current count buy? ----
% Bench data: steady-load current std 33-69 mA (motors 2-4); RoboClaw reports
% in 10 mA steps. Take sigma_I = 50 mA per channel.
sigI  = 0.050;                                % [A]
sGrid = 0.05:0.05:0.50;
dTds  = zeros(size(sGrid));
for k = 1:numel(sGrid)
    dTds(k) = (torqueAt(Wavg, sGrid(k) + 0.01, P) - torqueAt(Wavg, sGrid(k) - 0.01, P))/0.02;
end
sigS_wheel = KtMean*sigI./dTds;               % per-wheel slip noise (1 sigma)
sigS_side  = sqrt(2)*KtMean*sigI./(2*dTds);   % summed side: noise adds in quadrature
fprintf('\nSlip noise (1 sigma) from %d mA current noise, W = %.1f N\n', sigI*1000, Wavg);
fprintf('%6s %10s %12s %12s\n', 's', 'dT/ds', 'per-wheel', 'per-side');
for k = 1:numel(sGrid)
    fprintf('%6.2f %10.2f %12.3f %12.3f\n', sGrid(k), dTds(k), sigS_wheel(k), sigS_side(k));
end

%% 5. Calibration mismatch: one Kt for a side vs each motor's own Kt -----
% Side current converted with the mean Kt when the two motors differ.
fprintf('\nTorque bias from using mean Kt instead of per-motor Kt: %.0f%% to %+.0f%%\n', ...
        100*(min(Kt)/KtMean - 1), 100*(max(Kt)/KtMean - 1));
T0 = torqueAt(Wavg, 0.2, P);
for m = 1:4
    sBias = slipFromTorque(Wavg, T0*KtMean/Kt(m), P) - 0.2;
    fprintf('  Motor %d at s = 0.20: slip bias %+.3f\n', m, sBias);
end
