%% GA with Pattern Search

clear;
tic;

%% Load data for environment and WEC-Mobile Characterization
data = readtable('HawaiiApril2025_b7000.xlsx');
latitude = round(data.lat, 9);  
longitude = round(data.long, 9);
SWH = round(data.SWH1, 9);
MWP = round(data.MWP1,9);

% Temporal variation
DateTime = data.time; 
time_index = data.index;
DateTime = datetime(DateTime, 'InputFormat', 'dd-MMM-yyyy HH:mm:ss');
unique_times = unique(DateTime);

load("F_velocity.mat"); % Gives F_velocity
F_SWH = scatteredInterpolant(latitude, longitude, time_index, SWH, 'linear', 'linear');
F_MWP = scatteredInterpolant(latitude, longitude, time_index, MWP, 'linear', 'linear');
load("F_Desal.mat"); % Gives F_Desal

%% Define parameters for Lower level controller
% Options
opts = optimoptions('patternsearch', ...
    'Display','final', ...
    'UseCompletePoll',true, ...
    'UseCompleteSearch',false, ...
    'PollMethod','GSSPositiveBasis2N', ...
    'AccelerateMesh',true, ...
    'MeshTolerance',1e-6, ...
    'StepTolerance',1e-8, ...
    'MaxFunctionEvaluations',3000);

lb = [-5,   -4000];
ub = [ 5,    0];

% Hard Constraint on tank
q_lo = 880;
q_hi = 905;
 
% Choose probe size
ds = 1000; % Step 1000 m in the direction of movement

%% GA Parameters
% Define start and finish locations 

% Sri Lanka
% sf_lat = 9.83233;
% sf_long = 80.666;

% % % Hawaii
sf_lat = 19.75;
sf_long = -156.25;

% % Maldives
% sf_lat = 4.163;
% sf_long = 73.5783;

% Barbados
% sf_lat = 13;
% sf_long = -59.6667;

% % Palau 1
% sf_lat = 7.49;
% sf_long = 134.78;

% Palau 2
% sf_lat = 7.33;
% sf_long = 134.333;

% Define mission domain parameters

% Get mission domain size from the data
lat_max = max(latitude);
lat_min = min(latitude);
long_max = max(longitude);
long_min = min(longitude);

% Distance threshold for waypoint generation
t_angle = 20; % Set the threshold for angle between the pairs
dist_diff_max = 40*1000; % Max Distance between waypoints (in meters) 
dist_diff_min = 4*1000; % Min Distance between waypoints (in meters)

% Define boat paremeters
tank_size = 800; % in Liters 1 gal
allowed_buffer = 700; % Buffer in liters
buffer = 50; % Random variable. Not important

% Define land mass regions row-wise in format (lat_min, lat_max, lon_min, lon_max)

% white_regions = [9.58333, 9.74933, 80.5, 80.75]; % Sri Lanka

% white_regions = [13.0833, 13.33, -59.75, -59.4167]; % Barbados

white_regions = [18.8283, 20.4053, -154.693, -156.187]; % Hawaii

% white_regions = [4.164, 4.179, 73.499, 73.5197]; % Maldives

% white_regions = [7.324, 7.822, 134.324, 134.822]; % Palau

% Define Genetic Algorithm Parameters
num_coords = 8; % Number of coordinate pairs to generate.
num_generations = 50; % Number of generations for evolution
pop_size = 30; % Population size for GA
mutation_rate = 0.20; % Probability of mutation
elite_count = 1;  % Number of best individuals to carry over unchanged
num_immigrants = 15;   % number of fresh random individuals to inject each generation

stagnation_counter = 0;
max_stagnation = 8; % The number of generations excess ran when the fitness does not improve anymore
previous_best_fitness = -inf;

% Store best fitness for each generation
best_fitness_values = zeros(num_generations, 1);

% Storage for all generations' data
all_generations_data = struct();

% Initialize fitness struct with correct fields
fitnessData = struct('Fitness', [], 'Latitude', [], 'Longitude', [], 'Time', [], 'Desal', [], 'Desal_Opt', [], 'HamiltonianTf', [], 'p1_opt', [], 'p2_opt', [], 'TroubleshootTable', []);
fitnessData = repmat(fitnessData, pop_size, 1);

% Initial population of waypoints
waypoint = pop_generation(lat_max, lat_min, long_max, long_min, num_coords, sf_lat, sf_long, pop_size, t_angle, dist_diff_max, dist_diff_min, white_regions);
disp('Initial pop generated. Starting GA evolution.');
delete(gcp('nocreate'));
pool = parpool(4);
%% Run GA for multiple generations
for gen = 1:num_generations
        fitnessData = repmat(struct('Fitness', [], 'Latitude', [], 'Longitude', [], ...
        'Time', [], 'Desal', [], 'Desal_Opt', [], 'HamiltonianTf', [], ...
        'p1_opt', [], 'p2_opt', []), pop_size, 1);
    % Evaluate fitness for each waypoint sequence
    parfor i = 1:pop_size
        dir_lat = waypoint(i,:,1);
        dir_long = waypoint(i,:,2);

        % Check path at full speed
        [time, total_desalination, term] = GAEvaluationFullSpeedvaryingtimeindex (dir_lat, dir_long, F_velocity, F_SWH, F_MWP, F_Desal, DateTime, unique_times);
        
        if total_desalination > (tank_size) || total_desalination < (tank_size - allowed_buffer)
            
            % Store fitness and corresponding waypoints for all paths
            fitnessData(i).Fitness = 0; % Assign 0 fitness to sub-optimal paths
            fitnessData(i).Latitude = dir_lat;
            fitnessData(i).Longitude = dir_long;
            fitnessData(i).Time = time;
            fitnessData(i).Desal = total_desalination;
            % Do not assign any results related to lower level controller
            fitnessData(i).HamiltonianTf = [];
            fitnessData(i).p1_opt = [];
            fitnessData(i).p2_opt = [];
            fitnessData(i).Desal_Opt = [];
            fitnessData(i).TroubleshootTable = [];

        elseif term==0 && total_desalination >= (tank_size - allowed_buffer) && total_desalination <= (tank_size)
            fitnessData(i).Desal = total_desalination; % Store non-optimal desalination
            
            % Run optimization for valid paths
            nonlcon = @(p) qBandConstraint(p, F_velocity, F_SWH, F_MWP, F_Desal, DateTime, ...
                                             dir_lat, dir_long, tank_size, buffer, q_lo, q_hi, ds);
            % Initial guess of co-states
            p = [-3; -3500]; 
            p_opt = patternsearch(@(p) shootingCost(p, F_velocity, F_SWH, F_MWP, F_Desal, DateTime, unique_times, dir_lat, dir_long, tank_size, buffer, ds), p, [],[],[],[], lb, ub, nonlcon, opts);
            [time, desal_opt, H_final,Troubleshoot_Table] = lowerSimFuncvaryingtimeindex_ES_with5kmprobe(dir_lat, dir_long, p_opt(1), p_opt(2), F_velocity, F_SWH, F_MWP, F_Desal, DateTime, tank_size, buffer, ds);

            % ---- ALWAYS define fitness_eval (guard NaNs/Inf) ----
            fitness_eval = -Inf;  % penalty default
            if isfinite(desal_opt) && isfinite(time) 
                if desal_opt <= tank_size 
                    fitness_eval = (-time + 0.5*desal_opt);
                else
                    fitness_eval = (-time + 0.5*desal_opt)*1e-3;
                end
            end

            % Store Hamiltonian and Co-States for valid solutions
            fitnessData(i).HamiltonianTf = H_final;
            fitnessData(i).p1_opt = p_opt(1);
            fitnessData(i).p2_opt = p_opt(2);
            fitnessData(i).Fitness = fitness_eval;
            fitnessData(i).Latitude = dir_lat;
            fitnessData(i).Longitude = dir_long;
            fitnessData(i).Time = time;
            fitnessData(i).Desal_Opt = desal_opt;
            fitnessData(i).TroubleshootTable = Troubleshoot_Table;

            if fitnessData(i).Desal_Opt == fitnessData(i).Desal
                fitnessData(i).Fitness = (-time + 0.5*desal_opt)*.8;
            end


        else
            fitnessData(i).Fitness = 0; % Assign 0 fitness to sub-optimal paths
            fitnessData(i).Latitude = dir_lat;
            fitnessData(i).Longitude = dir_long;
            fitnessData(i).Time = time;
            fitnessData(i).Desal = total_desalination;
            % Do not assign any results related to lower level controller
            fitnessData(i).HamiltonianTf = [];
            fitnessData(i).p1_opt = [];
            fitnessData(i).p2_opt = [];
            fitnessData(i).Desal_Opt = [];
        end
        
    end

    % Sort by fitness descending
    [~, sorted_idx] = sort([fitnessData.Fitness], 'descend');

    % Store all path's data for this generation
    all_generations_data(gen).Generation = gen;
    all_generations_data(gen).FitnessData = fitnessData;

    % Select best solution
    [~, best_idx] = max([fitnessData.Fitness]);
    best_fitness_values(gen) = fitnessData(best_idx).Fitness;

    % Stagnation check
    current_best_fitness = best_fitness_values(gen);
    if abs(current_best_fitness - previous_best_fitness) < 1e-6
        stagnation_counter = stagnation_counter + 1;
    else
        stagnation_counter = 0;
    end
    previous_best_fitness = current_best_fitness;

    % Save data 
    save("Hawaii_PS_4_900L.mat", "all_generations_data");

    if stagnation_counter >= max_stagnation
        disp(['Stopping early at generation ', num2str(gen), ' due to stagnation.']);
        break;
    end

    % A waypoint is "valid" if it has coords and a finite fitness
    hasCoords = arrayfun(@(s) ~isempty(s.Latitude) && ~isempty(s.Longitude), fitnessData);
    hasFit    = arrayfun(@(s) isfinite(s.Fitness), fitnessData);
    validMask = hasCoords & hasFit;
    
    % Keep only VALID elites
    elite_pool = sorted_idx(validMask(sorted_idx));           % valid, best→worst
    elite_pool = elite_pool(1:min(elite_count, numel(elite_pool)));
    elites = waypoint(elite_pool, :, :);
    
    % Indices we never want to breed from (worst tail)
    worst_tail = sorted_idx(end-num_immigrants+1:end);
    
    % Allowed parents = valid & not from the excluded tail
    allowed_parent_idx = find(validMask);
    allowed_parent_idx = setdiff(allowed_parent_idx, worst_tail);
    
    % ---- Compute how many slots we must fill this generation -----------------
    imm_fixed = num_immigrants;                    % your regular immigrants
    breed_slots = pop_size - elite_count - imm_fixed;
    
    % If not enough valid parents to breed, increase immigrants to cover gap
    min_parents_needed = 2;                        % to run tournament selection
    if numel(allowed_parent_idx) < min_parents_needed
        % No safe breeding – fill ALL non-elite spots with immigrants
        imm_needed = pop_size - numel(elites);
        immigrants = pop_generation(lat_max, lat_min, long_max, long_min, ...
            num_coords, sf_lat, sf_long, imm_needed, ...
            t_angle, dist_diff_max, dist_diff_min, white_regions);
    
        new_population = zeros(pop_size, num_coords, 2);
        % place elites (if any)
        if ~isempty(elites)
            new_population(1:numel(elites),:,:) = elites;
        end
        % fill the rest with immigrants
        new_population(numel(elites)+1:end,:,:) = immigrants;
        waypoint = new_population;
        % Go to next generation
        continue
    end
    
    % Else: We have enough parents to breed.
    % Still, if parents are few, top off some slots with extra immigrants
    max_children_we_can_safely_make = breed_slots; % tournament can reuse parents
    extra_imm = 0;                                 % keep 0 unless you want a floor
    imm_total = imm_fixed + extra_imm;
    
    % ---- Build the next population ------------------------------------------
    new_population = zeros(pop_size, num_coords, 2);
    
    % 1) Elites
    if ~isempty(elites)
        new_population(1:elite_count,:,:) = elites;
    end
    
    % 2) Fixed immigrants
    immigrants = pop_generation(lat_max, lat_min, long_max, long_min, ...
        num_coords, sf_lat, sf_long, imm_total, ...
        t_angle, dist_diff_max, dist_diff_min, white_regions);
    
    imm_start = elite_count + 1;
    imm_end   = elite_count + imm_total;
    if imm_total > 0
        new_population(imm_start:imm_end,:,:) = immigrants;
    end
    
    % 3) Children using ONLY allowed valid parents
    child_start = imm_end + 1;
    for ii = child_start:pop_size
        p1 = tournament_selection(fitnessData, allowed_parent_idx);
        p2 = tournament_selection(fitnessData, allowed_parent_idx);
    
        child = crossover(p1, p2, sf_lat, sf_long);
        child = mutate(child, lat_max, lat_min, long_max, long_min, ...
                       mutation_rate, sf_lat, sf_long, white_regions);
    
        % final sanity: if child is bad, replace with a fresh immigrant
        if isempty(child) || any(~isfinite(child(:)))
            tmp = pop_generation(lat_max, lat_min, long_max, long_min, ...
                  num_coords, sf_lat, sf_long, 1, ...
                  t_angle, dist_diff_max, dist_diff_min, white_regions);
            child = squeeze(permute(tmp,[3 2 1]));
        end
    
        new_population(ii,:,:) = permute(child,[3 2 1]);
    end
    
    waypoint = new_population;  % ← next generation
end

%% Extract best fitness data accross generations
% Initialize arrays to store results
max_fitness_per_generation = [];
time_taken_per_generation = [];
latitude_per_generation = {};  % Store as a cell array to keep matrix format
longitude_per_generation = {}; % Store as a cell array to keep matrix format
p1_opt_per_gen = {};
p2_opt_per_gen = {};
Hamiltonian_per_gen = {};

% Loop through each generation
for i = 1:size(all_generations_data, 2)  % Iterate over each generation
    generation_data = all_generations_data(i).FitnessData; % Access FitnessData
    
    % Extract fitness values
    fitness_values = cellfun(@(x) x(1), {generation_data.Fitness});
    
    % Find the index of the max fitness
    [max_fitness, idx] = max(fitness_values);
    
    % Store the max fitness
    max_fitness_per_generation(i) = max_fitness;
    
    % Extract corresponding time, latitude, and longitude
    time_taken_per_generation(i) = generation_data(idx).Time; 
    latitude_per_generation{i} = generation_data(idx).Latitude;  % Store as matrix
    longitude_per_generation{i} = generation_data(idx).Longitude; % Store as matrix
    p1_opt_per_gen{i} = generation_data(idx).p1_opt;
    p2_opt_per_gen{i} = generation_data(idx).p2_opt;
    Hamiltonian_per_gen{i} = generation_data(idx).HamiltonianTf;
end

%% Get the overall maximum fitness and corresponding details
[overall_max_fitness, gen_idx] = max(max_fitness_per_generation);
least_time_taken = time_taken_per_generation(gen_idx);
dir_lat = latitude_per_generation{gen_idx};  % Keep as matrix
dir_long = longitude_per_generation{gen_idx}; % Keep as matrix
p_1_init = p1_opt_per_gen{gen_idx};
p_2 = p2_opt_per_gen{gen_idx};
Hamil_final = Hamiltonian_per_gen{gen_idx};

% Display the results
disp(['Overall Maximum Fitness: ', num2str(overall_max_fitness)]);
disp(['Time Taken: ', num2str(least_time_taken)]);
disp('Latitude Matrix:');
disp(dir_lat);
disp('Longitude Matrix:');
disp(dir_long);

% Save Best Coordinates Data
save("Hawaii_PS_4_ccords_900L.mat", 'least_time_taken', 'dir_lat', 'dir_long', 'p_1_init', "p_2","Hamil_final");

%% Plot fitness evolution
figure;
plot(1:size(all_generations_data, 2), max_fitness_per_generation, '-o', 'LineWidth', 2);
xlabel('Generation', 'Interpreter','latex', 'FontSize',25);
ylabel('Best Fitness', 'Interpreter','latex', 'FontSize',25);
% ylim([1.86, 1.88]);
title('Fitness Evolution Over Generations', 'Interpreter','latex', 'FontSize',25);
grid on;

toc;

%% Shooting cost function

% Cost function 
function J = shootingCost(p_guess, F_velocity, F_SWH, F_MWP, F_Desal, DateTime, unique_times, dir_lat, dir_long, tank_size, buffer, ds)
    qmax = tank_size;
    [timeFinal, qmission, Hfinal] = lowerSimFuncvaryingtimeindex_ES_with5kmprobe(dir_lat, dir_long, p_guess(1), p_guess(2), F_velocity, F_SWH, F_MWP, F_Desal, DateTime, tank_size, buffer, ds);
    
    J = timeFinal;
    if ~isfinite(J), J = 1000;
    end

end


%% Hard constraint on tank function
function [c, ceq] = qBandConstraint(p, F_velocity, F_SWH, F_MWP, F_Desal, DateTime, dir_lat, dir_long, tank_size, buffer, q_lo, q_hi, ds)
    try
        [t, q, H, ~] = lowerSimFuncvaryingtimeindex_ES_with5kmprobe( ...
            dir_lat, dir_long, p(1), p(2), F_velocity, F_SWH, F_MWP, F_Desal, DateTime, tank_size, buffer, ds);
    catch
        c = [1e6; 1e6];
        ceq = [];
        return
    end

    if ~isfinite(q)
        c = [0; 0];
        ceq = [];
        return
    end

    c   = [q_lo - q;   % <=0 means q >= q_lo
           q - q_hi];  % <=0 means q <= q_hi
    ceq = [];
end

%% Tournament Selection Function 
function parent = tournament_selection(fitnessData, allowed_idx)
    if nargin < 2 || isempty(allowed_idx)
        pool = 1:length(fitnessData);
    else
        pool = allowed_idx(:).';
    end
    tournament_size = 2;
    rp = randperm(numel(pool), tournament_size);
    pick = pool(rp);
    [~, best_local] = max([fitnessData(pick).Fitness]);
    best_idx = pick(best_local);
    parent = [fitnessData(best_idx).Latitude; fitnessData(best_idx).Longitude];
end

%% Crossover Function (Preserving First Waypoint)
function child = crossover(parent1, parent2, sf_lat, sf_long)
    mask = rand(size(parent1)) > 0.5;
    child = parent1 .* mask + parent2 .* (~mask);
    
    % Ensure first and last coordinates remains unchanged
    child(:,1) = [sf_lat; sf_long];
    child(:,end) = [sf_lat; sf_long];
end

%% Mutation Function (Preserving First Waypoint and Avoiding Land Regions + Distance Constraint)
function mutated = mutate(child, lat_max, lat_min, long_max, long_min, mutation_rate, sf_lat, sf_long, ...
    white_regions)
    % Constants for distance constraint (in meters)
    dist_diff_max = 40*1000;%20 * 1000; % 90 km
    dist_diff_min = 4 * 1000; % 20 km

    num_points = size(child, 2);

    if rand < mutation_rate
        mutation_idx = randi([2, num_points - 1]); % Avoid mutating the first and last (fixed) points

        prev_lat = child(1, mutation_idx - 1);
        prev_lon = child(2, mutation_idx - 1);
        next_lat = child(1, mutation_idx + 1);
        next_lon = child(2, mutation_idx + 1);

        valid_point = false;
        max_attempts = 100; % To avoid infinite loops
        attempt = 0;

        while ~valid_point && attempt < max_attempts
            attempt = attempt + 1;

            % Generate new random coordinate
            new_lat = lat_min + (lat_max - lat_min) * rand;
            new_lon = long_min + (long_max - long_min) * rand;

            % Check land exclusion
            if is_in_white_region(new_lat, new_lon, white_regions)
                continue; % Invalid due to land
            end

            % Compute distances to previous and next waypoints
            d_prev = haversine(prev_lat, prev_lon, new_lat, new_lon);
            d_next = haversine(new_lat, new_lon, next_lat, next_lon);

            if d_prev >= dist_diff_min && d_prev <= dist_diff_max && ...
               d_next >= dist_diff_min && d_next <= dist_diff_max
                valid_point = true;
                child(1, mutation_idx) = new_lat;
                child(2, mutation_idx) = new_lon;
            end
        end
    end

    % Preserve first and last coordinates
    child(:,1) = [sf_lat; sf_long];
    child(:,end) = [sf_lat; sf_long];
    mutated = child;
end

