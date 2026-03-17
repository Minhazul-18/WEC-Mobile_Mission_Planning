%% Mission Simulation Codes

clc;
clear;

%% Load data for environment and WEC-Mobile Characterization
data = readtable("HawaiiApril2025_b7000.xlsx"); % If data is .mat format, only load the data
latitude = round(data.lat, 9);  
longitude = round(data.long, 9);
SWH = round(data.SWH1, 9);
MWP = round(data.MWP1,9);

% Temporal variation
DateTime = data.time; 
time_index = data.index;
DateTime = datetime(DateTime, 'InputFormat', 'dd-MMM-yyyy HH:mm:ss');
unique_times = unique(DateTime);

% Load the WEC-Mpbile speed and desalination functions based on the Model
load("F_velocity.mat"); % Gives F_velocity
load("F_Desal.mat"); % Gives F_Desal
F_SWH = scatteredInterpolant(latitude, longitude, time_index, SWH, 'linear', 'linear');
F_MWP = scatteredInterpolant(latitude, longitude, time_index, MWP, 'linear', 'linear');

% Some Parameters relevant
tank_size = 900;
buffer = 50;
ds = 1000; % 1000 m step

% Load coordinates from the mission planner
load("Hawaii_PS_4_ccords_900L.mat");
savefile = "Hawaii_PS_4_ccords_900L_fullData_Sim.mat";


% Define code breaking parameter for infeasible guesses
too_long = 24*60*60* (7); % Bracketed term =  No. of days maximum you want to trial

% Total distance calculator
total_path_distance = 0;
for i = 1:(length(dir_lat)-1)
    distance = haversine(dir_lat(i), dir_long(i), dir_lat(i+1), dir_long(i+1));
    total_path_distance = total_path_distance + distance;
end

% Initial location
path_lat = dir_lat(1);
path_long = dir_long(1);

% Integration of co-state
p_1_array = [p_1_init];
df_ds_series = [0];
if p_1_init < 0
    logic = 1; % Start with boat moving at maximum speed
elseif p_1_init>=0
    logic = 2;
end

% Time Step
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
if logic == 1
    segment_velocities_2 = [F_velocity(SWH_series_2, MWP_series_2)]; % Effective speed of the boat
    speed_tracker = [1];
elseif logic == 2
    segment_velocities_2 = [0];
    speed_tracker = [0];
end
velocity_series_2 = [F_velocity(SWH_series_2, MWP_series_2)]; % Actual speed dictated by the sea state
latitude_series_2 = [path_lat];
longitude_series_2 = [path_long];
dateTime_series_2 = [DateTime(1)];


% Counters for code
direction_count = 2; % Counter for direction or lat2, lon2. Start with 2
dir2 = 1;
i = 1; % Initialize iter as 1
stuck_counter = 0;

% Initialize variables for mission
simulation_time_2 = 0;
total_desalination_2 = 0;
total_distance_2 = 0;
total_time_2 = 0;


%% Simulate boat movement

while true
switch logic
    case 1 % Boat moves at maximum speed
    for i = i:1:length(DateTime)

        if dir2 == length(dir_lat) || total_time_2>too_long
            % disp ('Done Done')
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
        speed_tracker = [speed_tracker;1];

        % Estimate the time point based on the simulation time
        dateTime_at_point = DateTime(1) + seconds(total_time_2);
        dateTime_series_2 = [dateTime_series_2; dateTime_at_point];
    
        % Calculation of co-state p_1
        ds_probe = ds; % 5 km
        if direction_count>length(dir_lat)
            stepper = direction_count-1;
        else
            stepper = direction_count;
        end
        [lat_f, lon_f] = revhaversine_step(path_lat, path_long, dir_lat(stepper), dir_long(stepper), +ds_probe);
        [lat_b, lon_b] = revhaversine_step(path_lat, path_long, dir_lat(stepper), dir_long(stepper), -ds_probe);
        % evaluate f at SAME time index (convert to SI if you use L/s)
        f_f = F_Desal(F_SWH(lat_f,    lon_f,    current_time_index), ...
                        F_MWP(lat_f,    lon_f,    current_time_index)) / 3600;
        f_b   = F_Desal(F_SWH(lat_b,    lon_b,    current_time_index), ...
                        F_MWP(lat_b,    lon_b,    current_time_index)) / 3600;
        
        df_ds = (f_f - f_b) / (2*ds_probe);   % (L/s)/m
        df_ds_series = [df_ds_series; df_ds];
        p_1 = p_1_array(end) - p_2 *df_ds*time_step;
        p_1_array = [p_1_array;p_1];

        % Calculate Remaining distance
        remaining_path = total_path_distance - total_distance_2;

        if i > 1
            if latitude_series_2(end) == latitude_series_2(end-1) && longitude_series_2(end) == longitude_series_2(end-1)
                stuck_counter = stuck_counter + 1;
            else
                stuck_counter = 0;  % Reset counter if movement is detected
            end
        end
    
        % Break the loop if stuck for 20 iterations
        if stuck_counter >= 10
            break;
        end

        if p_1_array(end) >= 0
            logic = 2;
            i=i+1;
            break;
        end
    end

    if dir2 == length(dir_lat) || total_time_2>too_long || stuck_counter >= 10
        break;
    end
   
    case 2 % Desalinate while remaining stationary
        for i= i:1:length(DateTime)

            swh_start = F_SWH(path_lat, path_long, current_time_index);
            mwp_start = F_MWP(path_lat, path_long, current_time_index);
            v_start = F_velocity(swh_start, mwp_start); % Station Keep
            desalination_value = F_Desal(swh_start, mwp_start);

            % Distance, Speed and Time
            segment_time_2 = time_step;
            small_distance = 15.67*(-1)^(i);
            [path_lat, path_long] = revhaversine_step(path_lat, path_long, dir_lat(dir2), dir_long(dir2), small_distance);
            segment_distance_2 = small_distance;
            effective_speed = 0;
            
            % Update total distance and time
            total_distance_2 = total_distance_2 + segment_distance_2;
            total_time_2 = total_time_2 + segment_time_2;
            simulation_time_2 = simulation_time_2 + segment_time_2;
            total_desalination_2 = total_desalination_2 + desalination_value * segment_time_2/3600;
            current_time_index = 1+ (total_time_2/(3*3600));

            % Store segment data
            segment_distances_2 = [segment_distances_2; segment_distance_2];
            segment_times_2 = [segment_times_2; segment_time_2];
            segment_velocities_2 = [segment_velocities_2; effective_speed];
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
            speed_tracker = [speed_tracker;0];
            

            % Estimate the time point based on the simulation time
            dateTime_at_point = DateTime(1) + seconds(total_time_2);
            dateTime_series_2 = [dateTime_series_2; dateTime_at_point];


            % Calculation of co-state p_1
            ds_probe = ds; % 5 km
            if direction_count>length(dir_lat)
                stepper = direction_count-1;
            else
                stepper = direction_count;
            end
            [lat_f, lon_f] = revhaversine_step(path_lat, path_long, dir_lat(stepper), dir_long(stepper), +ds_probe);
            [lat_b, lon_b] = revhaversine_step(path_lat, path_long, dir_lat(stepper), dir_long(stepper), -ds_probe);
            % evaluate f at SAME time index (convert to SI if you use L/s)
            f_f = F_Desal(F_SWH(lat_f,    lon_f,    current_time_index), ...
                            F_MWP(lat_f,    lon_f,    current_time_index)) / 3600;
            f_b   = F_Desal(F_SWH(lat_b,    lon_b,    current_time_index), ...
                            F_MWP(lat_b,    lon_b,    current_time_index)) / 3600;
            
            df_ds = (f_f - f_b) / (2*ds_probe);   % (L/s)/m
            df_ds_series = [df_ds_series; df_ds];
            p_1 = p_1_array(end) - p_2 *df_ds*time_step;
            p_1_array = [p_1_array;p_1];

            % Calculate Remaining distance
            remaining_path = total_path_distance - total_distance_2;        


        if p_1_array(end) < 0
            logic = 1;
            i=i+1;
            break;
        end

        if i == length(DateTime) || total_time_2>too_long 
            break;
        end   
        end

    if i == length(DateTime) || total_time_2>too_long 
        total_desalination_2 = 0; % Penalty for not reaching Base
        break;
    end 
end

end


% Convert to hours and kms
mission_time = total_time_2/3600;
mission_length = total_distance_2/1000;
avg_speed = total_distance_2/total_time_2;
avg_desalrate = total_desalination_2/mission_time;


% Evaluation of Hamiltonian at final time
H_final = 1+(segment_velocities_2(end)*p_1_array(end))+(p_2*(desal_rate_2(end)/3600));

% Final results
disp(['Total distance covered: ', num2str(mission_length), ' kilometers']);
disp(['Total time taken: ', num2str(mission_time), ' hours']);
disp(['Total desalination: ', num2str(total_desalination_2), ' Liters']);
disp(['Average speed along the path: ', num2str(avg_speed), ' m/s']);
disp(['Average desalination rate along the path: ', num2str(avg_desalrate), ' L/hr']);
disp(['Hamiltonian at the final time: ', num2str(H_final)]);

toc;