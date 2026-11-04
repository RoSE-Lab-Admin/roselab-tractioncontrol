%% RoSEy Terramechanics Model
% Single-wheel force calculator (W, DP, T), optimal slip lookup table,
% and onboard slip estimator prototype (Newton-Raphson).
%
% Model: Ding et al. 2009 (IROS), Eqs. 16-22
%   Normal stress:  Bekker pressure-sinkage with Wong-Reece stress distribution
%   Shear stress:   Janosi-Hanamoto
%   Grousers:       shearing radius r_s and lug-entry angle theta_f' (no F_passive)
%
% Units: SI throughout (m, N, Pa, rad) unless a comment says otherwise.
% Author: Ayooluwa Adeyinka, RoSE Lab, Colorado School of Mines
%
% Sections:
%   1. Parameters
%   2. Verification run against Ding's test wheel
%   3. Force vs slip curves for RoSEy
%   4. Optimal slip lookup table (and C++ header export)
%   5. Slip estimator demo
% Octave version: functions are defined first (Octave requires definition before use).

1;  % marks this file as a script so Octave allows function definitions below

%% ======================= FUNCTIONS ======================================

function N = sinkExp(s, P)
% Sinkage exponent, Ding 2009 Eq. 16
    if P.useSlipSinkage
        N = P.n0 + P.n1*s;
    else
        N = P.nStatic;
    end
end

function sig = normalStress(th, thf, s, P)
% Normal stress sigma(theta) [Pa], Ding 2009 Eq. 20 (Bekker + Wong-Reece)
    N    = sinkExp(s, P);
    thr  = P.c3*thf;
    thm  = (P.c1 + P.c2*s)*thf;
    kEq  = (P.kc/P.b + P.kphi)*1e3*P.r^N;     % kPa -> Pa
    sig  = zeros(size(th));
    front = th >= thm;                         % theta_m <= theta <= theta_f
    sig(front) = kEq*max(cos(th(front)) - cos(thf), 0).^N;
    rear = ~front;                             % theta_r <= theta < theta_m
    thEq = thf - (th(rear) - thr)./(thm - thr).*(thf - thm);
    sig(rear) = kEq*max(cos(thEq) - cos(thf), 0).^N;
end

function tau = shearStress(th, thf, s, sig, P)
% Shear stress tau(theta) [Pa], Janosi-Hanamoto with shearing radius (Ding 2009 Eq. 21).
% Shear displacement starts at theta_f' = acos((r - z)/Rj), where the lug tips
% enter the soil (Ding 2009 Eq. 17). Validation showed this term is required:
% without it DP RMSE vs Ding's Wh2 data is ~11 N; with it ~3 N.
    if P.useThetaPrime
        z   = P.r*(1 - cos(thf));
        th1 = acos(min((P.r - z)/P.Rj, 1));
    else
        th1 = thf;
    end
    j   = P.rs*((th1 - th) - (1 - s).*(sin(th1) - sin(th)));   % shear displacement [m]
    tau = (P.c + sig.*tan(P.phi)).*(1 - exp(-j./P.K));
end

function [W, DP, T, FT, R] = wheelForces(thf, s, P)
% Vertical load W [N], drawbar pull DP [N], torque T [N m],
% gross tractive force FT [N], compaction resistance R [N]. Ding 2009 Eq. 22.
    thr = P.c3*thf;
    th  = linspace(thr, thf, P.nInt + 1);
    sig = normalStress(th, thf, s, P);
    tau = shearStress(th, thf, s, sig, P);
    W   = P.b*simpson(P.r*sig.*cos(th) + P.rs*tau.*sin(th), th);
    FT  = P.b*simpson(P.rs*tau.*cos(th), th);
    R   = P.b*simpson(P.r*sig.*sin(th), th);
    DP  = FT - R;
    T   = P.rs^2*P.b*simpson(tau, th);
end

function I = simpson(f, x)
% Composite Simpson's rule on evenly spaced points (number of intervals must be even)
    n = numel(x) - 1;
    h = (x(end) - x(1))/n;
    I = h/3*(f(1) + 4*sum(f(2:2:n)) + 2*sum(f(3:2:n-1)) + f(end));
end

function thf = solveEntryAngle(Wtarget, s, P)
% Bisection: find entry angle theta_f such that W(theta_f, s) = Wtarget
    lo = 1e-4;  hi = P.thfMax;
    if wheelForces(hi, s, P) < Wtarget
        thf = NaN;  return;                    % load beyond model range
    end
    for it = 1:60
        mid = 0.5*(lo + hi);
        if wheelForces(mid, s, P) > Wtarget, hi = mid; else, lo = mid; end
        if hi - lo < 1e-8, break; end
    end
    thf = 0.5*(lo + hi);
end

function C = slipCurves(W, sGrid, P)
% Force vs slip curves at a fixed wheel load
    n = numel(sGrid);
    C.thf = NaN(1,n); C.DP = C.thf; C.T = C.thf; C.FT = C.thf; C.R = C.thf;
    for k = 1:n
        thf = solveEntryAngle(W, sGrid(k), P);
        C.thf(k) = thf;
        [~, C.DP(k), C.T(k), C.FT(k), C.R(k)] = wheelForces(thf, sGrid(k), P);
    end
end

function sx = firstCrossing(s, y, target)
% Smallest s where y(s) >= target, linearly interpolated. NaN if never reached.
    k = find(y >= target, 1, 'first');
    if isempty(k), sx = NaN; return; end
    if k == 1,     sx = s(1); return; end
    sx = s(k-1) + (target - y(k-1))*(s(k) - s(k-1))/(y(k) - y(k-1));
end

function [s, thf, ok, it] = estimateSlip(Wm, Tm, s0, thf0, P)
% Newton-Raphson on two equations: W(thf, s) = Wm and T(thf, s) = Tm.
% Finite-difference Jacobian, Cramer's rule, step limits, clamping, fail-safe.
    x  = [thf0; s0];
    ok = false;
    tolW = 1e-3*max(abs(Wm), 1);
    tolT = 1e-3*max(abs(Tm), 1e-3);
    d    = [1e-5; 1e-4];                       % finite-difference steps [rad; -]
    maxStep = [deg2rad(5); 0.1];
    for it = 1:20
        [W, ~, T] = wheelForces(x(1), x(2), P);
        Rv = [W - Wm; T - Tm];
        if abs(Rv(1)) < tolW && abs(Rv(2)) < tolT
            ok = true;  break;
        end
        [W1, ~, T1] = wheelForces(x(1) + d(1), x(2), P);
        [W2, ~, T2] = wheelForces(x(1), x(2) + d(2), P);
        J = [(W1 - W)/d(1), (W2 - W)/d(2);
             (T1 - T)/d(1), (T2 - T)/d(2)];
        detJ = J(1,1)*J(2,2) - J(1,2)*J(2,1);
        if abs(detJ) < 1e-10*norm(J(:,1))*norm(J(:,2))
            break;                             % singular: hold last good estimate
        end
        dx = -[ J(2,2), -J(1,2);
               -J(2,1),  J(1,1)]*Rv/detJ;      % Cramer's rule
        dx = max(min(dx, maxStep), -maxStep);  % limit step size
        x  = x + dx;
        x(1) = min(max(x(1), 1e-4), P.thfMax); % clamp to physical range
        x(2) = min(max(x(2), 0), 0.95);
    end
    thf = x(1);  s = x(2);
end

function s = kinematicSlip(omega, v, P)
% Kinematic slip from wheel speed and vehicle speed (IMU cross-check)
    vw = P.rs*omega;
    if vw == 0 && v == 0, s = 0;
    elseif vw >= v,       s = (vw - v)/vw;     % driving
    else,                 s = (vw - v)/v;      % skidding
    end
end

function exportHeader(fname, Wgrid, alphaGrid, sLow, sHigh)
% Write the lookup table as const C++ arrays for the Teensy (NaN -> -1 = infeasible)
    fid = fopen(fname, 'w');
    fprintf(fid, '// Auto-generated by rosey_terramechanics.m. Do not edit by hand.\n');
    fprintf(fid, '#pragma once\n\n');
    fprintf(fid, 'constexpr int N_LOAD = %d;\nconstexpr int N_ALPHA = %d;\n\n', numel(Wgrid), numel(alphaGrid));
    fprintf(fid, 'constexpr float LOAD_GRID_N[N_LOAD] = {%s};\n', fmtList('%.2ff', Wgrid));
    fprintf(fid, 'constexpr float ALPHA_GRID_DEG[N_ALPHA] = {%s};\n', fmtList('%.2ff', alphaGrid));
    fprintf(fid, 'constexpr float S_MAX[N_LOAD] = {%s};\n\n', fmtList('%.4ff', sHigh(:)'));
    fprintf(fid, '// S_MIN[load][incline], -1 marks an infeasible combination\n');
    fprintf(fid, 'constexpr float S_MIN[N_LOAD][N_ALPHA] = {\n');
    sl = sLow;  sl(isnan(sl)) = -1;
    for i = 1:numel(Wgrid)
        fprintf(fid, '    {%s},\n', fmtList('%.4ff', sl(i,:)));
    end
    fprintf(fid, '};\n');
    fclose(fid);
end

function out = fmtList(fmt, v)
% Comma-separated formatted list (Octave has no compose())
    out = strjoin(arrayfun(@(x) sprintf(fmt, x), v, 'UniformOutput', false), ', ');
end

%% ======================= MAIN SCRIPT =====================================
clc; close all;

%% 1. PARAMETERS ---------------------------------------------------------
% ---- Wheel: RoSEy (Hopkins dissertation) ----
P.r      = 0.1515;          % wheel radius [m]
P.b      = 0.05158;         % wheel width [m]
P.h      = 0.013;           % grouser height [m]
P.lam_s  = 0.5;             % lug shearing coefficient [-] (Ding 2009, s4 simplification)
P.rs     = P.r + P.lam_s*P.h;   % shearing radius [m]
P.useThetaPrime = true;     % lug-entry angle in shear displacement (Ding 2009 Eq. 17)
P.Rj     = P.r + P.h;       % lug tip radius used for theta_f' [m]

% ---- Soil: Ding et al. 2011 Table 2 (Bekker form) ----
% Fitted n0, n1, K from Ding et al. 2009, wheel Wh2 (closest radius to RoSEy).
% For MLSS runs, swap in Jehn's CSM-LHT-T-30 c, phi, rho (see parameter sheet).
P.kc     = 15.6;            % cohesive modulus of sinkage [kPa/m^(n-1)]
P.kphi   = 2407.4;          % frictional modulus of sinkage [kPa/m^n]
P.useSlipSinkage = true;    % true: N = n0 + n1*s (Ding 2009); false: N = nStatic
P.n0     = 0.81;            % static sinkage exponent [-]
P.n1     = 1.16;            % slip-sinkage coefficient [-]
P.nStatic = 1.10;           % plate-test sinkage exponent [-] (Ding 2011)
P.c      = 250;             % cohesion [Pa] (0.25 kPa)
P.phi    = deg2rad(31.9);   % internal friction angle [rad]
P.K      = 0.011;           % shear deformation modulus [m] (Ding 2011 range: 9.7-13.1 mm)
P.c1     = 0.5;             % max stress angle coefficient [-] (Ding 2009)
P.c2     = -0.3;            % max stress angle slip coefficient [-] (Ding 2009)
P.c3     = 0;               % exit angle constant, theta_r = c3*theta_f (Ding 2009)

% ---- Vehicle: RoSEy ----
P.m        = 16.01;         % total mass [kg] (Hopkins dataset config, CONFIRM)
P.g        = 9.81;          % [m/s^2]
P.nWheels  = 4;

% ---- Numerics ----
P.nInt     = 60;            % Simpson intervals (must be even)
P.thfMax   = deg2rad(80);   % upper bound for entry angle search [rad]

% ---- Torque input ----
% Wheel torque is published by existing rover code. The estimator expects
% wheel-side torque (after the gearbox) in N*m, one value per wheel.

fprintf('Shearing radius r_s = %.1f mm\n', P.rs*1e3);
fprintf('Static load per wheel = %.1f N\n\n', P.m*P.g/P.nWheels);

%% 2. VERIFICATION: Ding's test wheel ------------------------------------
% Reproduce the DP vs slip trend for Ding's Wh2 wheel at 80 N and compare
% against Ding et al. 2009 Fig. 6. Full scored comparison: rosey_validation.m
Pd = P;
Pd.r  = 0.15735;  Pd.b = 0.165;  Pd.h = 0.010;
Pd.rs = Pd.r + Pd.lam_s*Pd.h;  Pd.Rj = Pd.r + Pd.h;
sV = 0:0.05:0.6;
DPv = zeros(size(sV));  Tv = DPv;  zv = DPv;
for i = 1:numel(sV)
    thf = solveEntryAngle(80, sV(i), Pd);
    [~, DPv(i), Tv(i)] = wheelForces(thf, sV(i), Pd);
    zv(i) = Pd.r*(1 - cos(thf));
end
figure('Name','Verification: Ding Wh2, 80 N');
subplot(1,3,1); plot(sV, DPv, 'o-'); grid on; xlabel('slip s'); ylabel('DP [N]'); title('Drawbar pull');
subplot(1,3,2); plot(sV, Tv,  'o-'); grid on; xlabel('slip s'); ylabel('T [N m]'); title('Torque');
subplot(1,3,3); plot(sV, zv*1e3,'o-'); grid on; xlabel('slip s'); ylabel('z [mm]'); title('Sinkage');

%% 3. FORCE VS SLIP CURVES FOR ROSEY -------------------------------------
sGrid = 0:0.02:0.9;
Wplot = [25 40 55];               % per-wheel loads to plot [N]
figure('Name','RoSEy single-wheel curves');
for k = 1:numel(Wplot)
    C = slipCurves(Wplot(k), sGrid, P);
    subplot(1,3,1); hold on; plot(sGrid, C.FT, 'DisplayName', sprintf('W = %d N', Wplot(k)));
    subplot(1,3,2); hold on; plot(sGrid, C.DP, 'DisplayName', sprintf('W = %d N', Wplot(k)));
    subplot(1,3,3); hold on; plot(sGrid, C.T,  'DisplayName', sprintf('W = %d N', Wplot(k)));
end
subplot(1,3,1); grid on; xlabel('slip s'); ylabel('F_T [N]'); title('Gross tractive force'); legend('Location','southeast');
subplot(1,3,2); grid on; xlabel('slip s'); ylabel('DP [N]');  title('Drawbar pull');
subplot(1,3,3); grid on; xlabel('slip s'); ylabel('T [N m]'); title('Driving torque');

%% 4. OPTIMAL SLIP LOOKUP TABLE ------------------------------------------
% Upper bound: optimal slip (choose metric below).
% Lower bound: smallest slip that still supplies this wheel's share of the
%              drawbar pull needed to hold speed on the incline:
%              DP_req = m*g*sin(alpha)/nWheels  (steady state, R_M neglected).
metric    = 'tractiveKnee';   % 'tractiveKnee' or 'peakDP'
kneeFrac  = 0.95;             % knee = slip where F_T reaches this fraction of its max
Wgrid     = 20:5:60;          % per-wheel normal load [N]
alphaGrid = 0:5:20;           % incline [deg]

sLow  = NaN(numel(Wgrid), numel(alphaGrid));
sHigh = NaN(numel(Wgrid), 1);
for i = 1:numel(Wgrid)
    C = slipCurves(Wgrid(i), sGrid, P);
    switch metric
        case 'tractiveKnee'
            idx = find(C.FT >= kneeFrac*max(C.FT), 1, 'first');
        case 'peakDP'
            [~, idx] = max(C.DP);
    end
    sHigh(i) = sGrid(idx);
    for j = 1:numel(alphaGrid)
        DPreq = P.m*P.g*sind(alphaGrid(j))/P.nWheels;
        sLow(i,j) = firstCrossing(sGrid, C.DP, DPreq);
        if sLow(i,j) > sHigh(i), sLow(i,j) = NaN; end   % infeasible: flag
    end
end

fprintf('Optimal slip lookup table (metric: %s)\n', metric);
fprintf('Rows: wheel load W [N]. Columns: incline [deg]. Entry: [s_min, s_max]\n');
fprintf('%8s', 'W\\alpha'); fprintf('%14d', alphaGrid); fprintf('\n');
for i = 1:numel(Wgrid)
    fprintf('%8d', Wgrid(i));
    for j = 1:numel(alphaGrid)
        if isnan(sLow(i,j)), fprintf('%14s', 'infeasible');
        else, fprintf('   [%.2f, %.2f]', sLow(i,j), sHigh(i)); end
    end
    fprintf('\n');
end

save('rosey_slip_table.mat', 'Wgrid', 'alphaGrid', 'sLow', 'sHigh', 'metric', 'P');
exportHeader('slip_table.h', Wgrid, alphaGrid, sLow, sHigh);
fprintf('\nSaved rosey_slip_table.mat and slip_table.h\n\n');

%% 5. SLIP ESTIMATOR DEMO ------------------------------------------------
% On the rover: Wm comes from the four-wheel force balance + IMU tilt, and
% Tm comes from the rover's published wheel torque [N*m, wheel side].
% Here both are synthesized from a known state with 1% noise to check that
% the Newton-Raphson estimator recovers the slip.
sTrue = 0.20;  Wtrue = 40;
thfTrue = solveEntryAngle(Wtrue, sTrue, P);
[Wm, ~, Tm] = wheelForces(thfTrue, sTrue, P);
Wm = Wm*(1 + 0.01*randn);   Tm = Tm*(1 + 0.01*randn);   % 1% noise (stands in for torque topic)

s0 = 0.10;  thf0 = solveEntryAngle(Wm, s0, P);          % initial guess
[sEst, thfEst, ok, nIt] = estimateSlip(Wm, Tm, s0, thf0, P);
fprintf('Slip estimator: true s = %.3f, estimated s = %.3f (converged: %d, iterations: %d)\n', ...
        sTrue, sEst, ok, nIt);
fprintf('                true thf = %.2f deg, estimated thf = %.2f deg\n', ...
        rad2deg(thfTrue), rad2deg(thfEst));


