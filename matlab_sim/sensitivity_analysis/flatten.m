function flatten_all_datasets
data_dir = '.';
files = dir(fullfile(data_dir, 'ekf_R*_Q*.mat'));
for k = 1:numel(files)
  f = fullfile(files(k).folder, files(k).name);
  S = load(f);
  if ~isfield(S,'data') || ~isa(S.data,'Simulink.SimulationData.Dataset')
    warning('Skip %s (no Dataset "data").', files(k).name);
    continue;
  end
  ds = S.data;

  % Extract needed signals
  [t, roll_true] = extract_ts(getElement(ds,'roll_true'));
  [~, roll_hat ] = extract_ts(getElement(ds,'roll_hat'));
  [~, K_phi    ] = extract_ts(getElement(ds,'K_phi'));
  [~, K_theta  ] = extract_ts(getElement(ds,'K_theta'));

  % Save as flat variables alongside original
  [p,n,e] = fileparts(f);
  out = fullfile(p, [n '_flat' e]);
  save(out, 't','roll_true','roll_hat','K_phi','K_theta','-v7');  % v7 (non-HDF5)
  fprintf('Wrote %s\n', out);
end

  function [t, y] = extract_ts(el)
    if isprop(el,'Values') && ~isempty(el.Values)
      ts = el.Values;
      t = ts.Time(:); y = ts.Data(:);
    elseif isa(el,'timeseries')
      t = el.Time(:); y = el.Data(:);
    else
      error('Unsupported element type.');
    end
  end
end
