function [lat_next, lon_next, direction_count, dir2] = revhaversine(lat1, lon1, lat2, lon2, step_distance, direction_count,dir2)
    R = 6371000; % Earth's radius in meters

    % Convert to radians
    lat1_rad = deg2rad(lat1);
    lon1_rad = deg2rad(lon1);
    lat2_rad = deg2rad(lat2);
    lon2_rad = deg2rad(lon2);

    % Compute angular distance between points
    delta = 2 * asin(sqrt(sin((lat2_rad - lat1_rad)/2)^2 + ...
                  cos(lat1_rad) * cos(lat2_rad) * sin((lon2_rad - lon1_rad)/2)^2));
    
    % Handle the case where we're already at the destination
    if delta <= 1e-4
        lat_next = lat2;
        lon_next = lon2;
        direction_count = direction_count+1;
        dir2 = dir2+1;
        return;
    end

    % Compute fraction of the total distance to step
    f = step_distance / (R * delta);  % step as a fraction of total angular distance

    % Interpolate using spherical linear interpolation (slerp)
    A = sin((1 - f) * delta) / sin(delta);
    B = sin(f * delta) / sin(delta);

    x = A * cos(lat1_rad) * cos(lon1_rad) + B * cos(lat2_rad) * cos(lon2_rad);
    y = A * cos(lat1_rad) * sin(lon1_rad) + B * cos(lat2_rad) * sin(lon2_rad);
    z = A * sin(lat1_rad) + B * sin(lat2_rad);

    lat_next = atan2(z, sqrt(x^2 + y^2));
    lon_next = atan2(y, x);

    % Convert back to degrees
    lat_next = rad2deg(lat_next);
    lon_next = rad2deg(lon_next);
end
