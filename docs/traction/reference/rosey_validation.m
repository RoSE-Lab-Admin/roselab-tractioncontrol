%% RoSEy Terramechanics Model: Validation Against Published Data
% Compares the single-wheel model (Ding et al. 2009 IROS, Eqs. 16-22) with
% measured drawbar pull, torque and sinkage for Ding's wheel Wh2.
%
% Reference data (vector-extracted from the PDF figures, about +/-0.1 N):
%   Ding, Gao, Deng, Yoshida, Nagatani, "Slip ratio for lugged wheel of
%   planetary rover in deformable soil: definition and estimation,"
%   IEEE/RSJ IROS 2009, pp. 3343-3348.
%     Fig. 6(a),(c): measured DP and T vs s2 (Wh2, W = 80 N, h = 15/10 mm)
%     Fig. 12(b):    measured sinkage vs s3 (h = 15/10 mm)
%   Wh2: r = 157.35 mm, b = 165 mm, 30 lugs. Soil: kc 15.6, kphi 2407.4,
%   c 0.25 kPa, phi 31.9 deg; Wh2 fit n0 0.81, n1 1.16, K 0.011 m.
%
% Three model variants are scored:
%   A  "Current script":  shear uses theta_f, lambda_s = 0.5 (s4 definition)
%   B  A + theta_f' term:  shear uses theta_f' = acos((r - z)/Rj), Rj = r + h
%   C  Ding-faithful:      B with Ding's measured lambda_s3 (0.75 / 0.65)
% Slip conversion between definitions follows Ding 2009 Eq. 8.
%
% Metrics: RMSE, mean absolute error, max |error|, R^2.
% Author: Ayooluwa Adeyinka, RoSE Lab, Colorado School of Mines

1;  % script marker so Octave accepts the functions below

%% ======================= FUNCTIONS ======================================

function sig = normalStress(th, thf, s, P)
    N    = P.n0 + P.n1*s;
    thr  = P.c3*thf;
    thm  = (P.c1 + P.c2*s)*thf;
    kEq  = (P.kc/P.b + P.kphi)*1e3*P.r^N;
    sig  = zeros(size(th));
    front = th >= thm;
    sig(front) = kEq*max(cos(th(front)) - cos(thf), 0).^N;
    rear = ~front;
    thEq = thf - (th(rear) - thr)./(thm - thr).*(thf - thm);
    sig(rear) = kEq*max(cos(thEq) - cos(thf), 0).^N;
end

function tau = shearStress(th, thf, s, sig, P)
% Janosi-Hanamoto. With P.useThetaPrime the shear displacement starts at
% theta_f' (where the lug tips enter the soil, Ding 2009 Eq. 17 and 21).
    if P.useThetaPrime
        z   = P.r*(1 - cos(thf));
        th1 = acos(min((P.r - z)/P.Rj, 1));
    else
        th1 = thf;
    end
    j   = P.rs*((th1 - th) - (1 - s).*(sin(th1) - sin(th)));
    tau = (P.c + sig.*tan(P.phi)).*(1 - exp(-j./P.K));
end

function [W, DP, T] = wheelForces(thf, s, P)
    th  = linspace(P.c3*thf, thf, P.nInt + 1);
    sig = normalStress(th, thf, s, P);
    tau = shearStress(th, thf, s, sig, P);
    W   = P.b*simpson(P.r*sig.*cos(th) + P.rs*tau.*sin(th), th);
    DP  = P.b*simpson(P.rs*tau.*cos(th) - P.r*sig.*sin(th), th);
    T   = P.rs^2*P.b*simpson(tau, th);
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
        if hi - lo < 1e-9, break; end
    end
    thf = 0.5*(lo + hi);
end

function s = convertSlip(sOld, lamOld, lamNew, r, h)
% Ding 2009 Eq. 8: slip defined with r_s' = r + lamOld*h  ->  r_s = r + lamNew*h
    s = zeros(size(sOld));
    pos = sOld >= 0;
    s(pos)  = sOld(pos)  + (lamNew - lamOld)/(r/h + lamOld)*(1 - sOld(pos));
    s(~pos) = sOld(~pos) + (lamNew - lamOld)/(r/h + lamOld)*(1 + sOld(~pos));
end

function [DP, T, z] = predict(s, P)
    DP = NaN(size(s)); T = DP; z = DP;
    for i = 1:numel(s)
        thf = solveEntryAngle(P.W, s(i), P);
        if isnan(thf), continue; end
        [~, DP(i), T(i)] = wheelForces(thf, s(i), P);
        z(i) = P.r*(1 - cos(thf));
    end
end

function M = scoreFit(meas, pred)
    k = ~isnan(meas) & ~isnan(pred);
    e = pred(k) - meas(k);
    M.rmse = sqrt(mean(e.^2));
    M.mae  = mean(abs(e));
    M.maxe = max(abs(e));
    M.r2   = 1 - sum(e.^2)/sum((meas(k) - mean(meas(k))).^2);
    M.n    = nnz(k);
end

%% ======================= MAIN SCRIPT =====================================
clc; close all;

%% 1. PUBLISHED DATA (Ding 2009, Wh2, W = 80 N) ---------------------------
% Measured at slip s2 (defined with r + h). NaN = point not in the figure.
D.s2 = [0 0.05 0.10 0.20 0.30 0.40 0.50 0.60];
% h = 15 mm  (Ding lambda_s3 = 0.75)
D.h(1).h     = 0.015;  D.h(1).lam3 = 0.75;
D.h(1).DP    = [-6.43  9.94 17.62 23.06 25.07 25.83 27.07 27.57];   % [N]   Fig. 6(a)
D.h(1).T     = [ 0.03  2.94  4.51  6.15  6.87  7.24  7.67  8.08];   % [N m] Fig. 6(c)
D.h(1).z     = [ 2.89  3.78  4.90  7.92 11.69 16.58 22.49 28.49];   % [mm]  Fig. 12(b)
% h = 10 mm  (Ding lambda_s3 = 0.65)
D.h(2).h     = 0.010;  D.h(2).lam3 = 0.65;
D.h(2).DP    = [-4.31  6.42 14.40 20.45 22.21 22.56   NaN 24.79];
D.h(2).T     = [ 0.36  2.30  4.04  5.30  6.29  6.52   NaN  7.37];
D.h(2).z     = [ 3.97  4.90  6.17  9.05 12.75 16.72   NaN 26.77];

%% 2. MODEL PARAMETERS (Wh2 + Ding soil) ----------------------------------
B.r = 0.15735;  B.b = 0.165;  B.W = 80;
B.kc = 15.6;  B.kphi = 2407.4;  B.n0 = 0.81;  B.n1 = 1.16;
B.c = 250;  B.phi = deg2rad(31.9);  B.K = 0.011;
B.c1 = 0.5;  B.c2 = -0.3;  B.c3 = 0;
B.nInt = 80;  B.thfMax = deg2rad(80);

variants = {'A: current script', 'B: + theta_f'' (Rj = r+h)', 'C: Ding-faithful (B + lambda_s3)'};
useTP    = [false true true];
useLam3  = [false false true];
qty      = {'DP', 'T', 'z'};
units    = {'N', 'N m', 'mm'};

R = struct();  out = [];
for v = 1:numel(variants)
    for k = 1:2
        P = B;  P.h = D.h(k).h;
        P.useThetaPrime = useTP(v);
        P.Rj = P.r + P.h;
        if useLam3(v), P.lam = D.h(k).lam3; else, P.lam = 0.5; end
        P.rs = P.r + P.lam*P.h;
        sModel = convertSlip(D.s2, 1.0, P.lam, P.r, P.h);   % s2 -> model slip
        [DPp, Tp, zp] = predict(sModel, P);
        pred = {DPp, Tp, zp*1e3};
        meas = {D.h(k).DP, D.h(k).T, D.h(k).z};
        for q = 1:3
            M = scoreFit(meas{q}, pred{q});
            R(v,k,q).M = M;  R(v,k,q).pred = pred{q};  R(v,k,q).s = sModel;
            out(end+1,:) = [v, D.h(k).h*1e3, q, M.rmse, M.mae, M.maxe, M.r2]; %#ok<AGROW>
        end
    end
end

%% 3. RESULTS TABLE --------------------------------------------------------
fprintf('Validation vs Ding et al. 2009 (Wh2, W = 80 N)\n');
fprintf('%-34s %4s %4s %8s %8s %8s %7s\n', 'Variant', 'h', 'Qty', 'RMSE', 'MAE', 'Max|e|', 'R^2');
for v = 1:numel(variants)
    for k = 1:2
        for q = 1:3
            M = R(v,k,q).M;
            fprintf('%-34s %4d %4s %8.2f %8.2f %8.2f %7.3f  [%s]\n', variants{v}, ...
                    D.h(k).h*1e3, qty{q}, M.rmse, M.mae, M.maxe, M.r2, units{q});
        end
    end
end

fprintf('\nPoint-by-point, h = 15 mm (measured / A / C):\n');
fprintf('%6s | %18s | %18s | %18s\n', 's2', 'DP [N]', 'T [N m]', 'z [mm]');
for i = 1:numel(D.s2)
    fprintf('%6.2f | %5.1f %5.1f %5.1f  | %5.2f %5.2f %5.2f  | %5.1f %5.1f %5.1f\n', D.s2(i), ...
        D.h(1).DP(i), R(1,1,1).pred(i), R(3,1,1).pred(i), ...
        D.h(1).T(i),  R(1,1,2).pred(i), R(3,1,2).pred(i), ...
        D.h(1).z(i),  R(1,1,3).pred(i), R(3,1,3).pred(i));
end

csvwrite('validation_metrics.csv', out);   % cols: variant, h[mm], qty(1 DP,2 T,3 z), RMSE, MAE, max, R2

%% 4. PLOTS ----------------------------------------------------------------
figure('Name', 'Validation: Ding Wh2, 80 N');
mk = {'ks', 'k^'};  ln = {'-', '--', '-'};  lc = {[0.6 0.6 0.6], [0.2 0.4 0.8], [0.85 0.3 0.1]};
for q = 1:3
    subplot(1,3,q); hold on; grid on;
    for k = 1:2
        meas = {D.h(k).DP, D.h(k).T, D.h(k).z};
        plot(D.s2, meas{q}, mk{k}, 'MarkerFaceColor', 'k', 'DisplayName', sprintf('Ding meas., h=%d', D.h(k).h*1e3));
        for v = [1 3]
            plot(D.s2, R(v,k,q).pred, ln{v}, 'Color', lc{v}, 'DisplayName', sprintf('%s, h=%d', variants{v}(1), D.h(k).h*1e3));
        end
    end
    xlabel('slip s_2 (as tested)'); ylabel(sprintf('%s [%s]', qty{q}, units{q}));
end
subplot(1,3,1); legend('Location', 'southeast');
