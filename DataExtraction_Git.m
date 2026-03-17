clc;
clear;

%% Codes for saving MFWAM Forecast SWH and MWP  Time Stamped Data to a table from nc file
% Dataset link - https://data.marine.copernicus.eu/product/GLOBAL_ANALYSISFORECAST_WAV_001_027/description

file = 'FileName.nc'; % MFWAM File nc
excel_filename = 'FileName.mat'; % Output file - choose as .mat or .csv

% NC File Commands
info = ncinfo(file);
disp({info.Variables.Name}); 

% variables
swh_data = ncread(file, 'VHM0');
mwp_data = ncread(file, 'VTM02');
longitude = ncread(file, 'longitude');
latitude = ncread(file, 'latitude');
time = ncread(file, 'time');  % Time in seconds since 1970-01-01

% Define the range of time indices to process
start_time_index = 1;  % Starting time index
end_time_index = 1;   % Ending time index : Fixate from reading the nc file (ncdisp(file) in command window to see)

% Preallocate cell arrays for the entire dataset
longitude_vec = [];
latitude_vec = [];
swh_vec = [];
mwp_vec=[];
time_vec = [];
index_vec = [];

% Loop through the desired time indices
for time_index = start_time_index:end_time_index
    % Extract the data slices for the current time index
    swh_slice = squeeze(swh_data(:, :, time_index));  % (longitude x latitude)
    mwp_slice = squeeze(mwp_data(:, :, time_index));
    % Time Conversion to a human-readable format
    human_time = datetime(time(time_index), 'ConvertFrom', 'posixtime', 'TimeZone', 'UTC');

    % Create temporary arrays for this time index
    num_lon = length(longitude);
    num_lat = length(latitude);

    lon_temp = zeros(num_lon * num_lat, 1);
    lat_temp = zeros(num_lon * num_lat, 1);
    swh_temp = zeros(num_lon * num_lat, 1);
    mwp_temp = zeros(num_lon * num_lat, 1);
    index_temp = time_index * ones(num_lon * num_lat, 1);  % Add index column

    % Fill the temporary arrays with data
    idx = 1;
    for lon_idx = 1:num_lon
        for lat_idx = 1:num_lat
            lon_temp(idx) = longitude(lon_idx);  % Append longitude
            lat_temp(idx) = latitude(lat_idx);    % Append latitude
            swh_temp(idx) = swh_slice(lon_idx, lat_idx);  % Append significant wave height
            mwp_temp(idx) = mwp_slice(lon_idx, lat_idx);  % Append mean wave period
            idx = idx + 1;
        end
    end

    % Append to the main dataset
    longitude_vec = [longitude_vec; lon_temp];
    latitude_vec = [latitude_vec; lat_temp];
    swh_vec = [swh_vec; swh_temp];
    mwp_vec = [mwp_vec; mwp_temp];
    time_vec = [time_vec; repmat(human_time, num_lon * num_lat, 1)];  % Append the human-readable time
    index_vec = [index_vec; index_temp];
end

% to a table
data = table(longitude_vec, latitude_vec, swh_vec, mwp_vec, time_vec, index_vec, ...
    'VariableNames', {'long', 'lat', 'SWH1', 'MWP1', 'time', 'index'});

% Final saving
save(excel_filename, "data");

% Comment out previous line and uncomment following lines if saving as a
% csv file
% writetable(data_table, excel_filename);
% disp(['All data saved to Excel file: ', excel_filename]);
