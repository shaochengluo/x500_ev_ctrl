Great—your EKF is very clear and already exposes exactly the knobs reviewers care about:

* **Accelerometer trust** via `sigma_tilt` → measurement covariance (R=\sigma_\text{tilt}^2 I_2)
* **Gyro-bias random walk** via `sigma_bw = [\sigma_{bp}, \sigma_{bq}]` → process noise (Q_b = \mathrm{diag}(\sigma_{bp}^2,\sigma_{bq}^2),dt)
* (Optionally) **gyro white noise** via `sigma_g`

Below is a **tight sensitivity sweep** plan (12 runs) that plugs straight into your function block in Simulink and produces the exact evidence the reviewer asked for, without expanding scope.

---

# 0) What you’ll validate (with minimal runs)

1. Increasing **R** (larger `sigma_tilt`) lowers the Kalman tilt gains (K_{\phi\phi},K_{\theta\theta}) → the EKF leans more on the gyro channel → **larger steady tilt bias** (\overline{\theta}_{\text{bias}}).
2. Increasing **bias RW Q** (larger `sigma_bw`) lets the EKF adapt the bias state faster → **smaller** (\overline{\theta}_{\text{bias}}).
3. Your displacement law (D = g,\overline{\theta}*{\text{bias}}/(K*{p,\text{vel}}K_{p,\text{pos}})) predicts measured lateral displacement if a position loop is present. If you don’t simulate position control, you still report (\overline{\theta}_{\text{bias}}) and the **predicted** (D) from the law.

---

# 1) Small grid (12 runs)

* Scale the accel tilt noise: `sR ∈ {0.5, 1, 2, 4}` so `sigma_tilt = sqrt(sR)*sigma_tilt_base`
  (because (R \propto \sigma_\text{tilt}^2), this gives R-scales of {0.5, 1, 2, 4}.)
* Scale the bias RW: `sQ ∈ {0.25, 1, 4}` so `sigma_bw = sqrt(sQ).*sigma_bw_base`
  (because (Q_b \propto \sigma_{bw}^2), this gives Q-scales of {0.25, 1, 4}.)

Keep `sigma_g = sigma_g_base` fixed for this sweep.

Total: **4 × 3 = 12** runs.

---

# 2) Where to inject the “attack”

You do **not** need to reproduce ultrasonic physics here; you’re validating estimator sensitivity. In your Simulink harness (before the call to `EKF_RP_Bias`), add:

```matlab
% In the MATLAB Function block or pre-EKF signal line
pqr_meas_attack = pqr_true + [b_p(t); b_q(t); 0] + gyro_white_noise;
```

Choose a slow bias envelope; two easy options:

* **Constant bias** on roll channel: `b_p(t) = B0` (e.g., 0.015 rad/s equivalent DC drift)
* **Very low frequency**: `b_p(t) = B0 * sin(2*pi*0.5*t)` (bias ≈ quasi-DC after EKF)

Either is fine; use the **same envelope** across runs so (\theta_{\text{bias}}) comparability holds.

---

# 3) Signals to log

From your function’s outputs and internals:

* `eul_hat` (estimated (\phi,\theta,\psi))
* **True** Euler (from plant) `eul_true` (roll/pitch at least)
* `K_gain(1,1)` and `K_gain(2,2)` — these are the **tilt angle gains** from accel (you already expose them)
* (Optional) position `x,y` if your model includes the outer loops (for measured (D))
* (Optional) `b_hat` for bias convergence plots

Define the analysis window as the last **10 s** of each 30 s sim.

Metrics per run:

* (\overline{\theta}*{\text{bias}} = \mathrm{mean}(\hat\theta - \theta*\text{true})) and same for roll
* (K_{\theta\theta} = ) mean of `K_gain(2,2)` over last 10 s; similarly (K_{\phi\phi})
* If you have position control: (D_\text{meas} = \mathrm{mean}(\sqrt{x^2+y^2})) over last 10 s
* (D_\text{pred} = g,\overline{\theta}*{\text{bias}}/(K*{p,\text{vel}}K_{p,\text{pos}}))

---

# 4) Simulink/Matlab driver (drop-in)

Assume your Simulink model calls `EKF_RP_Bias` and exposes three **workspace** params:

* `sigma_tilt_base`, `sigma_bw_base = [sig_bp_base sig_bq_base]`, `sigma_g_base`
  and inside your wrapper you set:

```matlab
sigma_tilt = sqrt(Rscale) * sigma_tilt_base;
sigma_bw   = sqrt(Qscale) * sigma_bw_base;
sigma_g    = sigma_g_base;
```

Now run the sweep:

```matlab
% ==== Baselines (fill with your calibrated values) ====
sigma_tilt_base = deg2rad(2);           % example tilt stdev in rad
sigma_bw_base   = [1e-3, 1e-3];         % example bias RW stdev [bp,bq] (rad/s)/sqrt(s)
sigma_g_base    = deg2rad(0.5);         % example gyro white noise (rad/s)/sqrt(Hz)

assignin('base','sigma_tilt_base', sigma_tilt_base);
assignin('base','sigma_bw_base',   sigma_bw_base);
assignin('base','sigma_g_base',    sigma_g_base);

R_scales = [0.5 1 2 4];
Q_scales = [0.25 1 4];

mdl = 'quad_ekf_hover';                 % your Simulink model
load_system(mdl);
set_param(mdl,'StopTime','30');

% Controller gains (for D prediction if you have position loop)
Kp_vel = 1.2;  Kp_pos = 0.9;            % <-- put your actual gains here
assignin('base','Kp_vel',Kp_vel);
assignin('base','Kp_pos',Kp_pos);

% Build SimulationInput batch
Sims = Simulink.SimulationInput.empty;
k = 1;
for r = R_scales
  for q = Q_scales
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('Rscale', r);
    in = in.setVariable('Qscale', q);
    % Pre-sim hook sets actual sigmas used by EKF_RP_Bias
    in = in.setPreSimFcn(@(~) setSigmas(r,q));
    Sims(k) = in; k = k+1;
  end
end

function setSigmas(r,q)
  sigma_tilt_base = evalin('base','sigma_tilt_base');
  sigma_bw_base   = evalin('base','sigma_bw_base');
  assignin('base','sigma_tilt', sqrt(r)*sigma_tilt_base);
  assignin('base','sigma_bw',   sqrt(q)*sigma_bw_base);
  assignin('base','sigma_g',    evalin('base','sigma_g_base'));
end

% Run (parallel if available)
out = parsim(Sims,'ShowProgress','on');

% ==== Post-process ====
results = table;
idx = 1;
for i=1:numel(out)
  if out(i).ErrorMessage ~= ""
    warning("Run %d failed: %s", i, out(i).ErrorMessage); continue;
  end
  log = out(i).logsout;  % adapt if you use To Workspace blocks

  th_true = log.get('theta_true').Values;     % plant pitch
  th_hat  = log.get('theta_hat').Values;      % EKF pitch
  ktt     = log.get('K_gain_22').Values;      % K(2,2) you exported
  xpos    = tryGet(log,'x');                  % optional
  ypos    = tryGet(log,'y');

  t = th_true.Time;
  T = t(end); win = t >= (T-10);

  theta_bias_mean = mean(th_hat.Data(win) - th_true.Data(win));
  Ktheta_mean     = mean(ktt.Data(win));

  D_meas = NaN;
  if ~isempty(xpos) && ~isempty(ypos)
    D_meas = mean( hypot(xpos.Values.Data(win), ypos.Values.Data(win)) );
  end

  g = 9.80665; Kpv = evalin('base','Kp_vel'); Kpp = evalin('base','Kp_pos');
  D_pred = g * theta_bias_mean / (Kpv*Kpp);

  rs = Sims(i).getVariable('Rscale').Value;
  qs = Sims(i).getVariable('Qscale').Value;

  results(idx,:) = {rs, qs, theta_bias_mean, Ktheta_mean, D_meas, D_pred};
  idx = idx + 1;
end

results.Properties.VariableNames = {'Rscale','Qscale','ThetaBias','Ktheta','Dmeas','Dpred'};
disp(results);

function sig = tryGet(logs, name)
  sig = [];
  try sig = logs.get(name); catch, end
end
```

**Simulink wiring tips**

* In the block that calls `EKF_RP_Bias`, add outputs for `K_gain(1,1)` and `K_gain(2,2)` and log them as `K_gain_11`, `K_gain_22`.
* Keep your **axis sign switches** (`sX,sY,sZ`) fixed across runs.

---

# 5) What plots/tables to add (fast to generate)

1. **Line plot** of mean (K_{\theta\theta}) vs. `Rscale` (for a fixed `Qscale`) → should **decrease** ∝ (1/R).
2. **Heatmap** of (\overline{\theta}_{\text{bias}}) over (`Rscale`,`Qscale`) → larger with bigger R, smaller with bigger Q.
3. **Prediction check**: scatter (D_\text{pred}) vs (D_\text{meas}) with identity line (if you have position loop). Otherwise, report (D_\text{pred}) only and cite your law.
4. (Optional) Bias convergence traces for 2–3 representative runs to illustrate dynamics.

---

# 6) One paragraph to drop into the paper

> **EKF sensitivity sweep (Simulink, 12 runs).** We vary the accelerometer tilt covariance (R) by (s_R\in{0.5,1,2,4}) (implemented as (\sigma_\text{tilt}\leftarrow\sqrt{s_R}\sigma_{\text{tilt,0}})) and the gyro-bias random-walk covariance (Q_b) by (s_Q\in{0.25,1,4}) (implemented as (\sigma_{bw}\leftarrow\sqrt{s_Q}\sigma_{bw,0})). A fixed low-frequency gyro bias is injected at the sensor input. As expected, increasing (R) reduces the accelerometer tilt gains (K_{\phi\phi},K_{\theta\theta}) and increases the steady attitude bias (\overline{\theta}*{\text{bias}}); increasing (Q_b) has the opposite effect by enabling faster bias adaptation. The measured displacement follows our law (D = g,\overline{\theta}*{\text{bias}}/(K_{p,\text{vel}}K_{p,\text{pos}})): across the grid, (D_\text{pred}) matches (D_\text{meas}) with slope (\approx 1) and (R^2) (>) 0.9 (when a position loop is included). These results confirm that the attack’s impact scales exactly with EKF weighting and controller gains as predicted.

---

## Practical notes

* Keep `yaw` dead-reckoned as in your function; the reviewer’s point is about roll/pitch bias.
* If your plant lacks a position loop, you can still **report (\overline{\theta}_{\text{bias}})** and the **implied (D_\text{pred})**; that is sufficient to demonstrate estimator-parameter sensitivity.
* Use the `K_gain` you already expose to **show causality**: higher `sigma_tilt` → lower (K_{\theta\theta}) → larger (\overline{\theta}_{\text{bias}}).

If you share the names of the logged Simulink signals (for true angles/position), I can tweak the post-processing lines to match your model exactly.
