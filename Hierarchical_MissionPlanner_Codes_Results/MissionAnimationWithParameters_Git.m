%% Code to animate mission with speed, desal rate and tank fill showing

% Load Mission Output Data First
load('FileName.mat'); % Mission Must be Run by mission Simulator first and data saved


%% Run this section to save mission animation on desal contour
%  Mission Animation: Desal contour + boat trail | Speed, Desal rate, Tank


gif_filename_mission = 'SaveFile.gif';
delay_seconds_anim   = 0.01;   % fast playback; increase for slower GIF
nLevels_anim         = 20;
frame_step           = 42; % Adjust based on data. Frame step must be a divisor of the total datapoints of the stored data

n_frames = length(time_series_2);

% ----------------------------------------------------------
%  Pre-compute fixed colour axis for desal contour
%  (sampled at the time indices the boat actually visits)
% ----------------------------------------------------------
desal_cmin = inf;  desal_cmax = -inf;

unique_tindex_visited = unique(floor(index_series_2));
unique_tindex_visited(unique_tindex_visited < 1) = 1;
unique_tindex_visited(unique_tindex_visited > length(unique_times)) = length(unique_times);

for k = unique_tindex_visited(:)'
    t_k    = unique_times(k);
    mask_k = (DateTime == t_k);
    if ~any(mask_k), continue; end
    dv = F_Desal(SWH(mask_k), MWP(mask_k));
    desal_cmin = min(desal_cmin, min(dv));
    desal_cmax = max(desal_cmax, max(dv));
end

% ----------------------------------------------------------
%  Fixed axis limits for time-series subplots
% ----------------------------------------------------------
t_hours      = time_series_2 / 3600;          % mission time in hours
xlim_t       = [0, max(t_hours)];

ylim_speed   = [0,   max(segment_velocities_2)*1.15 + 1e-6];
ylim_desal   = [0,   max(desal_rate_2)        *1.15 + 1e-6];
ylim_tank    = [0,   max(desalination_series_2)*1.15 + 1e-6];

% Map limits
xlim_map = [min(longitude) max(longitude)];
ylim_map = [min(latitude)  max(latitude)];

% Shade regions by logic (speed tracker): 1 = moving, 0 = stationary
switch_times   = t_hours(logical([1; diff(speed_tracker)]));   % hours where mode changed
switch_tracker = speed_tracker(logical([1; diff(speed_tracker)]));

% ----------------------------------------------------------
%  Build figure
% ----------------------------------------------------------
fig_anim = figure('Color', 'w');
set(fig_anim, 'Position', [50 50 1600 800]);

cached_tidx  = -1;   % track which contour is currently drawn
cached_lgrid = [];
cached_lond  = [];
cached_latd  = [];

for fi = 1:frame_step:n_frames

    % ---- Which environmental time slice to show? ----
    tidx = max(1, min(length(unique_times), floor(index_series_2(fi))));

    % ---- Rebuild contour only when time slice changes ----
    if tidx ~= cached_tidx
        t_env    = unique_times(tidx);
        mask_env = (DateTime == t_env);

        if any(mask_env)
            lat_e = latitude (mask_env);
            lon_e = longitude(mask_env);
            lat_u = unique(lat_e);
            lon_u = unique(lon_e);

            if numel(lat_u) >= 2 && numel(lon_u) >= 2
                [cached_lond, cached_latd] = meshgrid(lon_u, lat_u);
                dv_e = F_Desal(SWH(mask_env), MWP(mask_env));
                cached_lgrid = griddata(lon_e, lat_e, dv_e, cached_lond, cached_latd);
                cached_tidx  = tidx;
            end
        end
    end

    clf(fig_anim);

    % ==================================================
    %  LEFT PANEL — map
    % ==================================================
    subplot(1, 4, [1 2]);   % map takes left half

    if ~isempty(cached_lgrid)
        contourf(cached_lond, cached_latd, cached_lgrid, nLevels_anim, ...
                 'LineColor', 'none');
        hold on;
        clim([desal_cmin desal_cmax]);
    else
        hold on;
    end

    cb = colorbar('FontSize', 14, 'Location', 'southoutside', ...
                  'TickLabelInterpreter', 'latex');
    cb.Label.String    = 'Desalination Rate (L/hr)';
    cb.Label.Interpreter = 'latex';
    cb.Label.FontSize  = 14;

    % Growing trail
    plot(longitude_series_2(1:fi), latitude_series_2(1:fi), ...
         'w-', 'LineWidth', 1.2, 'HandleVisibility', 'off');

    % Waypoints
    for w = 1:length(dir_lat)-1
        plot(dir_long(w), dir_lat(w), 'ks', 'MarkerSize', 8, ...
             'MarkerFaceColor', 'w', 'HandleVisibility', 'off');
        text(dir_long(w), dir_lat(w), sprintf(' %d', w), ...
             'FontSize', 10, 'Color', 'w', 'Clipping', 'on', ...
             'VerticalAlignment', 'bottom', 'Interpreter', 'none');
    end

    % Boat dot — colour by mode (red = moving, blue = stationary)
    if speed_tracker(fi) == 1
        boat_color = 'r';
        mode_str   = 'Transiting';
    else
        boat_color = 'b';
        mode_str   = 'Station-keeping';
    end

    plot(longitude_series_2(fi), latitude_series_2(fi), '.', ...
         'Color', boat_color, 'MarkerSize', 22, 'DisplayName', mode_str);

    legend('Location', 'northwest', 'FontSize', 11, ...
           'Interpreter', 'latex', 'TextColor', 'w', 'Color', 'none', ...
           'EdgeColor', 'w');

    xlim(xlim_map); ylim(ylim_map);
    xlabel('Longitude ($^\circ$)', 'FontSize', 14, 'Interpreter', 'latex');
    ylabel('Latitude ($^\circ$)',  'FontSize', 14, 'Interpreter', 'latex');
    title(sprintf('WEC-Mobile mission — %s', datestr(dateTime_series_2(fi), ...
          'dd-mmm-yyyy HH:MM')), 'FontSize', 13, 'Interpreter', 'latex');
    set(gca, 'FontSize', 12, 'TickLabelInterpreter', 'latex');
    hold off;

    % ==================================================
    %  RIGHT PANELS — time series (3 × 1)
    % ==================================================

    % ---- (a) Effective speed ----
    subplot(4, 4, [3 4]);
    plot(t_hours(1:fi), segment_velocities_2(1:fi), 'b-', 'LineWidth', 1.5);
    xlim(xlim_t); ylim(ylim_speed);
    ylabel('Speed (m/s)', 'FontSize', 12, 'Interpreter', 'latex');
    title('(a) Effective speed', 'FontSize', 12, 'Interpreter', 'latex');
    set(gca, 'FontSize', 11, 'TickLabelInterpreter', 'latex', 'XTickLabel', []);
    grid on;

    % ---- (b) Desalination rate ----
    subplot(4, 4, [7 8]);
    plot(t_hours(1:fi), desal_rate_2(1:fi), 'g-', 'LineWidth', 1.5);
    xlim(xlim_t); ylim(ylim_desal);
    ylabel('Desal. rate (L/hr)', 'FontSize', 12, 'Interpreter', 'latex');
    title('(b) Desalination rate', 'FontSize', 12, 'Interpreter', 'latex');
    set(gca, 'FontSize', 11, 'TickLabelInterpreter', 'latex', 'XTickLabel', []);
    grid on;

    % ---- (c) Tank fill ----
    subplot(4, 4, [11 12]);
    plot(t_hours(1:fi), desalination_series_2(1:fi), 'r-', 'LineWidth', 1.5);
    % Tank capacity reference line
    yline(tank_size, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    xlim(xlim_t); ylim(ylim_tank);
    ylabel('Tank (L)', 'FontSize', 12, 'Interpreter', 'latex');
    xlabel('Mission time (hr)', 'FontSize', 12, 'Interpreter', 'latex');
    title('(c) Tank fill', 'FontSize', 12, 'Interpreter', 'latex');
    set(gca, 'FontSize', 11, 'TickLabelInterpreter', 'latex');
    grid on;

    drawnow;

    % ---- Capture and write GIF frame ----
    frame     = getframe(fig_anim);
    img       = frame2im(frame);
    [A, cmap] = rgb2ind(img, 256);

    if fi == 1
        imwrite(A, cmap, gif_filename_mission, 'gif', ...
                'LoopCount', inf, 'DelayTime', delay_seconds_anim);
    else
        imwrite(A, cmap, gif_filename_mission, 'gif', ...
                'WriteMode', 'append', 'DelayTime', delay_seconds_anim);
    end
end
 
% Un-comment to save gir
% disp(['Mission animation saved to: ', gif_filename_mission]);

%% Run to see mission on a Geographic map
%  Mission Animation: Geographic map + boat trail | Speed, Desal rate, Tank
% =========================================================

gif_filename_mission = 'SaveFile.gif';
delay_seconds_anim   = 0.001;
frame_step           = 42; % Adjust based on data. Frame step must be a divisor of the total datapoints of the stored data

n_frames = length(time_series_2);
frame_indices = [1:frame_step:n_frames, n_frames];
% ----------------------------------------------------------
%  Fixed axis limits for time-series subplots
% ----------------------------------------------------------
t_hours    = time_series_2 / 3600;
xlim_t     = [0, max(t_hours)];
ylim_speed = [0, max(segment_velocities_2)*1.15 + 1e-6];
ylim_desal = [0, max(desal_rate_2)        *1.15 + 1e-6];
ylim_tank  = [0, max(desalination_series_2)*1.15 + 1e-6];

% ----------------------------------------------------------
%  Load map data once (outside the loop)
% Adjust based on location
land = shaperead('landareas', 'UseGeoCoords', true, ...
    'BoundingBox', [-158, 18; -155, 21]);   % Hawaii

% ----------------------------------------------------------
%  Build figure layout
% ----------------------------------------------------------
fig_anim = figure('Color', 'w');
set(fig_anim, 'Position', [50 50 1600 800]);

% ---- Geographic axes (left half) ----
ax_map = subplot(1, 4, [1 2]);
ax_map = worldmap([18, 21], [-158, -155]);
setm(ax_map, 'Grid', 'on', 'GLineStyle', '-', 'GColor', [0.7 0.7 0.7], ...
     'MLineLocation', 0.5, 'PLineLocation', 0.5);
setm(ax_map, 'FontSize', 12);

% Land layer (static — drawn once)
geoshow(ax_map, land, ...
    'FaceColor', [0.8 0.7 0.6], 'EdgeColor', [0.5 0.4 0.3], ...
    'LineWidth', 1, 'HandleVisibility', 'off');

% Waypoints (static — drawn once)
for w = 1:length(dir_lat)-1
    geoshow(ax_map, dir_lat(w), dir_long(w), ...
        'DisplayType', 'point', ...
        'Marker', 's', 'MarkerSize', 8, ...
        'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k', ...
        'HandleVisibility', 'off');
    textm(dir_lat(w), dir_long(w), sprintf(' %d', w), ...
        'FontSize', 12, 'Color', 'k', 'Clipping', 'on', ...
        'VerticalAlignment', 'bottom');
end

% Placeholder handles for dynamic elements (trail + boat dot)
h_trail = [];
h_boat  = [];

% ---- Right-panel axes (created once, reused each frame) ----
ax_speed = subplot(4, 4, [3  4 ]);
ax_desal = subplot(4, 4, [7  8 ]);
ax_tank  = subplot(4, 4, [11 12]);

% ----------------------------------------------------------
%  Animation loop
% ----------------------------------------------------------
for fi = frame_indices

% ---- Update trail and boat ----
    if ~isempty(h_trail) && isvalid(h_trail), delete(h_trail); end
    if ~isempty(h_boat)  && isvalid(h_boat),  delete(h_boat);  end

    axes(ax_map);   % <-- make the map axes current before plotm

    h_trail = plotm(latitude_series_2(1:fi), longitude_series_2(1:fi), ...
                    '-', 'Color', [0.1 0.1 0.1], 'LineWidth', 1.5);

    if speed_tracker(fi) == 1
        boat_color = 'r';
    else
        boat_color = [0 0.45 0.74];
    end
    h_boat = plotm(latitude_series_2(fi), longitude_series_2(fi), ...
                   '.', 'MarkerSize', 22, 'Color', boat_color);
    % ---- Map title ----
    title(ax_map, ...
        sprintf('Mission path (%s)', ...
            datestr(dateTime_series_2(fi), 'dd-mmm-yyyy HH:MM')), ...
        'FontSize', 14, 'Interpreter', 'latex');

    % ---- (a) Effective speed ----
    cla(ax_speed);
    plot(ax_speed, t_hours(1:fi), segment_velocities_2(1:fi), ...
         'b-', 'LineWidth', 1.5);
    xlim(ax_speed, xlim_t); ylim(ax_speed, ylim_speed);
    ylabel(ax_speed, 'Speed (m/s)',      'FontSize', 12, 'Interpreter', 'latex');
    title(ax_speed,  '(a) Speed', 'FontSize', 12, 'Interpreter', 'latex');
    set(ax_speed, 'FontSize', 12, 'TickLabelInterpreter', 'latex', 'XTickLabel', []);
    grid(ax_speed, 'on');

    % ---- (b) Desalination rate ----
    cla(ax_desal);
    plot(ax_desal, t_hours(1:fi), desal_rate_2(1:fi), ...
         'g-', 'LineWidth', 1.5);
    xlim(ax_desal, xlim_t); ylim(ax_desal, ylim_desal);
    ylabel(ax_desal, 'Desal. rate (L/hr)', 'FontSize', 12, 'Interpreter', 'latex');
    title(ax_desal,  '(b) Desalination rate', 'FontSize', 12, 'Interpreter', 'latex');
    set(ax_desal, 'FontSize', 12, 'TickLabelInterpreter', 'latex', 'XTickLabel', []);
    grid(ax_desal, 'on');

    % ---- (c) Tank fill ----
    cla(ax_tank);
    plot(ax_tank, t_hours(1:fi), desalination_series_2(1:fi), ...
         'r-', 'LineWidth', 1.5);
    yline(ax_tank, tank_size, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    xlim(ax_tank, xlim_t); ylim(ax_tank, ylim_tank);
    ylabel(ax_tank, 'Tank (L)',           'FontSize', 12, 'Interpreter', 'latex');
    xlabel(ax_tank, 'Mission time (hr)',  'FontSize', 12, 'Interpreter', 'latex');
    title(ax_tank,  '(c) Tank fill',      'FontSize', 12, 'Interpreter', 'latex');
    set(ax_tank, 'FontSize', 12, 'TickLabelInterpreter', 'latex');
    grid(ax_tank, 'on');

    drawnow;

    % ---- Capture and write GIF frame ----
    frame     = getframe(fig_anim);
    img       = frame2im(frame);
    [A, cmap] = rgb2ind(img, 256);

    if fi == 1
        imwrite(A, cmap, gif_filename_mission, 'gif', ...
                'LoopCount', inf, 'DelayTime', delay_seconds_anim);
    else
        imwrite(A, cmap, gif_filename_mission, 'gif', ...
                'WriteMode', 'append', 'DelayTime', delay_seconds_anim);
    end
end

% Un - Comment to save ouput
%disp(['Mission animation saved to: ', gif_filename_mission]);