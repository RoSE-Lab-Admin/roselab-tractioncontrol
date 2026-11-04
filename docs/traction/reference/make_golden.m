%% make_golden.m: golden values for the C++ port of the RoSEy terramechanics model
% Same model as rosey_terramechanics_octave.m (validated variant: theta_f' lug-entry
% term, Rj = r + h, lambda_s = 0.5). Writes CSV files the C++ unit tests compare against.
% Run:  octave-cli make_golden.m
1;
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
function [W, DP, T, FT, R] = wheelForces(thf, s, P)
    th = linspace(P.c3*thf, thf, P.nInt + 1);
    sig = normalStress(th, thf, s, P);  tau = shearStress(th, thf, s, sig, P);
    W  = P.b*simpson(P.r*sig.*cos(th) + P.rs*tau.*sin(th), th);
    FT = P.b*simpson(P.rs*tau.*cos(th), th);
    R  = P.b*simpson(P.r*sig.*sin(th), th);
    DP = FT - R;
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
        if hi - lo < 1e-10, break; end
    end
    thf = 0.5*(lo + hi);
end

% ---- RoSEy wheel + Ding soil (same as main script) ----
R.r = 0.1515; R.b = 0.05158; R.h = 0.013; R.lam_s = 0.5;
R.kc = 15.6; R.kphi = 2407.4; R.n0 = 0.81; R.n1 = 1.16;
R.c = 250; R.phi = deg2rad(31.9); R.K = 0.011;
R.c1 = 0.5; R.c2 = -0.3; R.c3 = 0; R.nInt = 60; R.thfMax = deg2rad(80);
R.rs = R.r + R.lam_s*R.h; R.Rj = R.r + R.h;
% ---- Ding Wh2 wheel (h = 15 mm), same soil ----
D = R; D.r = 0.15735; D.b = 0.165; D.h = 0.015; D.rs = D.r + D.lam_s*D.h; D.Rj = D.r + D.h;

% 1) Direct force evaluation at fixed (thf, s): tests the integrals alone
fid = fopen('golden_forces.csv', 'w');
fprintf(fid, 'wheel,thf_rad,s,W_N,DP_N,T_Nm,FT_N,R_N\n');
names = {'rosey', 'ding_wh2'};  Ps = {R, D};
for w = 1:2
    for thf = deg2rad([10 20 30 40])
        for s = [0 0.1 0.2 0.4 0.6]
            [W, DP, T, FT, Rr] = wheelForces(thf, s, Ps{w});
            fprintf(fid, '%s,%.10f,%.4f,%.10g,%.10g,%.10g,%.10g,%.10g\n', names{w}, thf, s, W, DP, T, FT, Rr);
        end
    end
end
fclose(fid);

% 2) Load-matched curves (bisection on thf): tests solveEntryAngle + forces
fid = fopen('golden_curves.csv', 'w');
fprintf(fid, 'wheel,W_target_N,s,thf_rad,z_m,DP_N,T_Nm\n');
loads = {[25 39.3 55], [80]};
for w = 1:2
    for Wt = loads{w}
        for s = 0:0.05:0.6
            thf = solveEntryAngle(Wt, s, Ps{w});
            [~, DP, T] = wheelForces(thf, s, Ps{w});
            fprintf(fid, '%s,%.4f,%.4f,%.10f,%.10g,%.10g,%.10g\n', names{w}, Wt, s, thf, Ps{w}.r*(1 - cos(thf)), DP, T);
        end
    end
end
fclose(fid);

% 3) Estimator cases: exact (W, T) from a known (thf, s); the C++ Newton solver
%    must recover s and thf from W and T.
fid = fopen('golden_estimator.csv', 'w');
fprintf(fid, 'W_N,T_Nm,s_true,thf_true_rad\n');
for Wt = [25 39.3 55]
    for s = [0.05 0.1 0.2 0.3 0.35]
        thf = solveEntryAngle(Wt, s, R);
        [W, ~, T] = wheelForces(thf, s, R);
        fprintf(fid, '%.10g,%.10g,%.4f,%.10f\n', W, T, s, thf);
    end
end
fclose(fid);
disp('wrote golden_forces.csv, golden_curves.csv, golden_estimator.csv');
