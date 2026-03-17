%% Haversine function to calculate distance between two lat/long pairs
function d = haversine(lat1, lon1, lat2, lon2)
    R = 6371000; % Earth's radius in meters
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlat/2) * sin(dlat/2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2) * sin(dlon/2);
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    d = R * c;
end