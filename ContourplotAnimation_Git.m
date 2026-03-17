clc;
clear all;

% =========================================================
%  Read Data
% =========================================================
data     = readtable("HawaiiApril2025_b7000.xlsx"); % If data is .mat format, only load the data
latitude  = round(data.lat,  9);
longitude = round(data.long, 9);
SWH       = round(data.SWH1, 9);
MWP       = round(data.MWP1, 9);

DateTime     = datetime(data.time, 'InputFormat', 'dd-MMM-yyyy HH:mm:ss');
unique_times = unique(DateTime);

% Load the WEC-Mpbile speed and desalination functions based on the Model
load("F_Desal.mat");
load("F_velocity.mat"); 

% Load output file from mission planner if visualization of path is
% required. Otherwise commented
% load("FileName.mat", "longitude_series_2", "latitude_series_2", "dir_lat","dir_long");

% =========================================================
%  GIF settings
% =========================================================
last_time_index = 20;            % number of frames
gif_filename    = 'FileName.gif';
delay_seconds   = 0.5;
nLevels         = 20;

xlim_all = [min(longitude) max(longitude)];
ylim_all = [min(latitude)  max(latitude)];

% =========================================================
%  Pre-compute global colour-axis limits (one pass per variable)
%  so colour scales stay fixed across all frames
% =========================================================
desal_min = inf;  desal_max = -inf;
SWH_min   = inf;  SWH_max   = -inf;
MWP_min   = inf;  MWP_max   = -inf;
speed_min = inf;  speed_max = -inf;

for k = 1:last_time_index
    t         = unique_times(k);
    mask      = (DateTime == t);
    if ~any(mask), continue; end

    dv = F_Desal   (SWH(mask), MWP(mask));
    sv = F_velocity(SWH(mask), MWP(mask));

    desal_min = min(desal_min, min(dv));
    desal_max = max(desal_max, max(dv));
    SWH_min   = min(SWH_min,   min(SWH(mask)));
    SWH_max   = max(SWH_max,   max(SWH(mask)));
    MWP_min   = min(MWP_min,   min(MWP(mask)));
    MWP_max   = max(MWP_max,   max(MWP(mask)));
    speed_min = min(speed_min, min(sv));
    speed_max = max(speed_max, max(sv));
end

% =========================================================
%  Helper: scatter → grid
% =========================================================
makeGrid = @(lon_s, lat_s, vals, lat_u, lon_u) ...
    griddata(lon_s, lat_s, vals, ...
             meshgrid(lon_u, lat_u), ...   % lon_grid
             meshgrid(lat_u, lon_u)');     % lat_grid  (transpose trick)

% =========================================================
%  Animation loop
% =========================================================
fig = figure('Color', 'w');
set(fig, 'Position', [100 100 1600 1000]);

for fi = 1:last_time_index

    t    = unique_times(fi);
    mask = (DateTime == t);
    if ~any(mask)
        warning('No data at frame %d – skipping.', fi);
        continue;
    end

    lat_s  = latitude (mask);
    lon_s  = longitude(mask);
    SWH_s  = SWH(mask);
    MWP_s  = MWP(mask);

    lat_u = unique(lat_s);
    lon_u = unique(lon_s);

    if numel(lat_u) < 2 || numel(lon_u) < 2
        warning('Not enough unique points at frame %d – skipping.', fi);
        continue;
    end

    % Build lon/lat grids once per frame (shared by all 4 panels)
    [lon_grid, lat_grid] = meshgrid(lon_u, lat_u);

    % Interpolate each variable onto the grid
    desal_grid = griddata(lon_s, lat_s, F_Desal   (SWH_s, MWP_s), lon_grid, lat_grid);
    SWH_grid   = griddata(lon_s, lat_s, SWH_s,                     lon_grid, lat_grid);
    MWP_grid   = griddata(lon_s, lat_s, MWP_s,                     lon_grid, lat_grid);
    speed_grid = griddata(lon_s, lat_s, F_velocity(SWH_s, MWP_s), lon_grid, lat_grid);

    clf(fig);
    time_str = datestr(t, 'dd-mmm-yyyy HH:MM:SS');

    % ----------------------------------------------------------
    %  Panel layout helper
    % ----------------------------------------------------------
panels = { ...
        SWH_grid,   [SWH_min   SWH_max  ], '(a) SWH (m)';                ...
        MWP_grid,   [MWP_min   MWP_max  ], '(b) MWP (s)';                ...
        desal_grid, [desal_min desal_max], '(c) Desalination Rate (L/hr)'; ...
        speed_grid, [speed_min speed_max ], '(d) Transit Speed (m/s)';    ...
    };

    for p = 1:4
        subplot(2, 2, p);

        grid_data = panels{p, 1};
        clim_p    = panels{p, 2};
        ttl       = panels{p, 3};

        contourf(lon_grid, lat_grid, grid_data, nLevels, ...
                 'LineColor', 'none', 'HandleVisibility', 'off');
        hold on;

        % Fixed colour axis
        clim(clim_p);
        % % Comment out if visualization of path is required. Otherwise
        % commented
%         % ---- Path overlay ----
%         plot(longitude_series_2, latitude_series_2, 'k-', 'LineWidth', 1);
% 
%         for i = 1:length(dir_long)-1
%             plot(dir_long(i), dir_lat(i), '.r', 'MarkerSize', 13, ...
%                  'HandleVisibility', 'off');
%             text(dir_long(i), dir_lat(i), sprintf('%d', i), ...
%                  'VerticalAlignment',   'bottom', ...
%                  'HorizontalAlignment', 'right',  ...
%                  'FontSize', 10, 'Color', 'red', 'Clipping', 'on');
%         end
% 
%         plot(longitude_series_2(1), latitude_series_2(1), 'r.', ...
%              'MarkerSize', 20, 'DisplayName', 'Start/Finish');

        % ---- Cosmetics ----
        cb = colorbar('FontSize', 18, 'Location', 'southoutside', ...
                      'TickLabelInterpreter', 'latex');
        cb.Label.Interpreter = 'latex';

        title([ttl, ' — ', time_str], 'FontSize', 14, 'Interpreter', 'latex');
        xlabel('Longitude ($^\circ$)', 'FontSize', 16, 'Interpreter', 'latex');
        ylabel('Latitude ($^\circ$)',  'FontSize', 16, 'Interpreter', 'latex');
        set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');

        xlim(xlim_all);
        ylim(ylim_all);
        hold off;
    end

    drawnow;

    % ---- Capture frame and write to GIF ----
    frame      = getframe(fig);
    img        = frame2im(frame);
    [A, cmap]  = rgb2ind(img, 256);

    if fi == 1
        imwrite(A, cmap, gif_filename, 'gif', ...
                'LoopCount', inf, 'DelayTime', delay_seconds);
    else
        imwrite(A, cmap, gif_filename, 'gif', ...
                'WriteMode', 'append', 'DelayTime', delay_seconds);
    end

end

% disp(['GIF saved to: ', gif_filename]); % Un-comment to save output