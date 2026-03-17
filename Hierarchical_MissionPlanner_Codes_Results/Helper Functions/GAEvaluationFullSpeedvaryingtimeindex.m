function [mission_time, total_desalination_2, term] = GAEvaluationFullSpeedvaryingtimeindex (dir_lat, dir_long, F_velocity, F_SWH, F_MWP, F_Desal, DateTime, unique_times)
close all;

% Modified function with dependency of speed and desalination directly on
% the SWH and MWP and not the location and time

too_long = 24*60*60* (7);
%% Boat move at full speed at all times

% Initial location
path_lat = dir_lat(1);
path_long = dir_long(1);

% Time variables
time_step = 240; % in secs
current_time_index = 1; % Starting time index (Put in the time step when you want the mission to start)

% Initialize Arrays for mission data storage
time_series_2 = [0]; % Accumulates time
desalination_series_2 = [0];
distance_series_2 = [0];
SWH_series_2 = [F_SWH(path_lat, path_long, current_time_index)];
MWP_series_2 = [F_MWP(path_lat, path_long, current_time_index)];
index_series_2 =[1];
desal_rate_2 = [F_Desal(SWH_series_2, MWP_series_2)];
segment_distances_2 = [0]; % Stores distance for each individual steps
segment_times_2 = [0]; % Stores time for each individual steps
segment_velocities_2 = [F_velocity(SWH_series_2, MWP_series_2)]; % Effective speed of the boat
velocity_series_2 = [F_velocity(SWH_series_2, MWP_series_2)]; % Actual speed dictated by the sea state
latitude_series_2 = [path_lat];
longitude_series_2 = [path_long];
dateTime_series_2 = [DateTime(1)];

% Counters
direction_count = 2; % Counter for direction or lat2, lon2. Start with 2
dir2 = 1;
i = 1; % Initialize iter as 1
stuck_counter = 0;  % Counter for detecting if the latitude is stuck i.e. struck land
term = 0; % Indication of boat hitting the land

% Initialize variables for mission
simulation_time_2 = 0;
total_desalination_2 = 0;
total_distance_2 = 0;
total_time_2 = 0;


%% Simulate boat movement-Move at maximum speed

for i = i:1:length(DateTime)

    if dir2 == length(dir_lat) || total_time_2>too_long
       break
    end

    % Use ScatteredInterpolant to find the velocity at the starting point of the segment
    swh_start = F_SWH(path_lat, path_long, current_time_index);
    mwp_start = F_MWP(path_lat, path_long, current_time_index);
    v_start = F_velocity(swh_start, mwp_start);
    
    % Handle NaN velocity values
    if isnan(v_start)
        v_start = 0;  % Assign mean value if NaN found
    end
    
    % Interpolate the desalination at the starting point of the segment
    desalination_value = F_Desal(swh_start, mwp_start);

    % Handle NaN desalination values
    if isnan(desalination_value)
        desalination_value = 0;  % Assign a mean value 
    end
    segment_time_2 = time_step;
    segment_distance_2 = v_start*time_step;
    [path_lat, path_long, direction_count, dir2] = revhaversine(path_lat, path_long, dir_lat(direction_count), dir_long(direction_count), segment_distance_2,direction_count,dir2);
    
    
    % Update total distance and time
    total_distance_2 = total_distance_2 + segment_distance_2;
    total_time_2 = total_time_2 + segment_time_2;
    simulation_time_2 = simulation_time_2 + segment_time_2;
    total_desalination_2 = total_desalination_2 + desalination_value * segment_time_2/3600;
    current_time_index = 1+ (total_time_2/(3*3600));

    % Store segment data
    segment_distances_2 = [segment_distances_2; segment_distance_2];
    segment_times_2 = [segment_times_2; segment_time_2];
    segment_velocities_2 = [segment_velocities_2; v_start];
    velocity_series_2 = [velocity_series_2; v_start];
    desalination_series_2 = [desalination_series_2; total_desalination_2];
    desal_rate_2 = [desal_rate_2; desalination_value];
    distance_series_2 = [distance_series_2; total_distance_2];
    time_series_2 = [time_series_2; total_time_2];
    index_series_2 = [index_series_2; current_time_index];
    SWH_series_2 = [SWH_series_2; swh_start];
    MWP_series_2 = [MWP_series_2; mwp_start];
    latitude_series_2 = [latitude_series_2; path_lat];
    longitude_series_2 = [longitude_series_2; path_long];

    if i > 1
        if latitude_series_2(end) == latitude_series_2(end-1) && longitude_series_2(end) == longitude_series_2(end-1)
            stuck_counter = stuck_counter + 1;
        else
            stuck_counter = 0;  % Reset counter if movement is detected
        end
    end

    % Break the loop if stuck for 20 iterations
    if stuck_counter >= 10
        disp('Terminating: Boat hit land.');
        term = 1; % Indicator of boat hitting the land
        break;
    end
    
    % Estimate the time point based on the simulation time
    dateTime_at_point = DateTime(1) + seconds(total_time_2);
    dateTime_series_2 = [dateTime_series_2; dateTime_at_point];

    % Change time index when 3 hours have passed
    % if simulation_time_2 >= (3*3600)*current_time_index && current_time_index < length(unique_times)
    %     current_time_index = current_time_index + 1;
    % end

    if dir2 == length(dir_lat) || total_time_2>too_long
        break;
    end
end

% Convert to hours and kms
mission_time = total_time_2/3600;
mission_length = total_distance_2/1000;
avg_speed = total_distance_2/total_time_2;
avg_desalrate = total_desalination_2/mission_time;


% Final results
disp(['------------------------------------------------------------------------------------------']);
disp(['Total distance covered: ', num2str(mission_length), ' kilometers']);
disp(['Total time taken: ', num2str(mission_time), ' hours']);
disp(['Total desalination: ', num2str(total_desalination_2), ' Liters']);
disp(['Average speed along the path: ', num2str(avg_speed), ' m/s']);
disp(['Average deslination rate along the path: ', num2str(avg_desalrate), ' L/hr']);
disp(['------------------------------------------------------------------------------------------']);

end