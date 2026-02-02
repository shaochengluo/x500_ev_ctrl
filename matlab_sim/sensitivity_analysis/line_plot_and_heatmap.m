function line_plot_and_heatmap
%% ========= User config =========
data_dir   = '.';                  % folder containing ekf_R*_Q*_flat.mat
R_scales   = [0.5 1 2 4];
Q_scales   = [0.25 1 4];

win_attack_steady = [20 30];       % seconds
fixed_Q_idx = 2;                   % which Q slice to show (2 -> Q=1)

%% ========= Discover files =========
files = dir(fullfile(data_dir, 'ekf_R*_Q*_flat.mat'));
if isempty(files)
  error('No files matched ekf_R*_Q*_flat.mat in %s', data_dir);
end

% Results table
res = table('Size',[0 6], ...
  'VariableTypes', {'double','double','double','double','double','double'}, ...
  'VariableNames', {'Rscale','Qscale','RollBias','Ktheta','Kphi','N'});

for k = 1:numel(files)
  f = files(k).name;

  % Parse R, Q from filename (R0p5 -> 0.5)
  tok = regexp(f, 'R(?<R>[0-9p\.]+)_Q(?<Q>[0-9p\.]+)_flat\.mat$', 'names');
  if isempty(tok)
    warning('Skipping %s: cannot parse R/Q from filename.', f); 
    continue;
  end
  Rscale = str2double(strrep(tok.R,'p','.'));
  Qscale = str2double(strrep(tok.Q,'p','.'));

  S = load(fullfile(files(k).folder, f));
  need = {'t','roll_true','roll_hat','K_phi','K_theta'};
  if ~all(isfield(S, need))
    warning('Skipping %s: missing one of %s.', f, strjoin(need, ', '));
    continue;
  end

  t         = S.t(:);
  roll_true = S.roll_true(:);
  roll_hat  = S.roll_hat(:);
  K_phi     = S.K_phi(:);
  K_theta   = S.K_theta(:);

  % Truncate to common length (defensive)
  L = min([numel(t), numel(roll_true), numel(roll_hat), numel(K_phi), numel(K_theta)]);
  t         = t(1:L);
  roll_true = roll_true(1:L);
  roll_hat  = roll_hat(1:L);
  K_phi     = K_phi(1:L);
  K_theta   = K_theta(1:L);

  % Window indices
  w = (t >= win_attack_steady(1)) & (t <= win_attack_steady(2));
  if ~any(w)
    warning('%s: no samples in [%g,%g] s window; skipped.', f, win_attack_steady(1), win_attack_steady(2));
    continue;
  end

  % Metrics
  roll_bias_mean = mean(roll_hat(w) - roll_true(w));
  Ktheta_mean    = mean(K_theta(w));
  Kphi_mean      = mean(K_phi(w));

  res(end+1,:) = {Rscale, Qscale, roll_bias_mean, Ktheta_mean, Kphi_mean, sum(w)};
end

if height(res) == 0
  error('No valid runs loaded.');
end

%% ========= Plot 1: mean K_theta vs Rscale (fixed Q) =========
q_val = Q_scales(fixed_Q_idx);
slice = res(res.Qscale == q_val, :);
[~,ord] = sort(slice.Rscale);
slice = slice(ord,:);

figure('Name','Ktheta vs Rscale'); clf;
plot(slice.Rscale, slice.Ktheta, '-o', 'LineWidth', 1.6, 'MarkerSize', 6);
grid on;
xlabel('Rscale (accelerometer tilt covariance multiplier)');
ylabel('mean K_{\theta\theta} during [20,30] s');
title(sprintf('K_{\\theta\\theta} vs Rscale (Qscale = %.3g)', q_val));
xlim([min(R_scales) max(R_scales)]);

%% ========= Plot 2: heatmap of roll bias over (Rscale, Qscale) =========
M = nan(numel(R_scales), numel(Q_scales));
for iR = 1:numel(R_scales)
  for iQ = 1:numel(Q_scales)
    rows = res.Rscale == R_scales(iR) & res.Qscale == Q_scales(iQ);
    if any(rows)
      M(iR,iQ) = mean(res.RollBias(rows));
    end
  end
end

figure('Name','Roll bias heatmap'); clf;
imagesc(Q_scales, R_scales, M);
set(gca,'YDir','normal');
xlabel('Qscale (gyro-bias process covariance multiplier)');
ylabel('Rscale (accelerometer tilt covariance multiplier)');
title('\bf mean roll bias (rad) during [20,30] s');
cb = colorbar; ylabel(cb,'rad');
set(gca,'XTick',Q_scales,'YTick',R_scales);

%% ========= Optional: print table =========
res = sortrows(res, {'Qscale','Rscale'});
disp('=== Summary (means during [20,30] s) ===');
disp(res);
end
