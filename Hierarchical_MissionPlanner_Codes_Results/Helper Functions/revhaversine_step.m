function [lat_out, lon_out] = revhaversine_step(lat1, lon1, lat2, lon2, ds_probe)
%REVHAVERSINE_STEP  Move ds_probe meters from (lat1,lon1) toward (lat2,lon2)
%using great-circle slerp, without mutating any external counters.
%
%  ds_probe > 0 : forward along track toward (lat2,lon2)
%  ds_probe < 0 : backward along track (away from waypoint)
%
% Inputs in degrees, ds_probe in meters. Outputs in degrees.

    R = 6371000; % m

    % radians
    lat1r = deg2rad(lat1); lon1r = deg2rad(lon1);
    lat2r = deg2rad(lat2); lon2r = deg2rad(lon2);

    % central angle
    delta = 2*asin( sqrt( sin((lat2r-lat1r)/2)^2 + ...
                     cos(lat1r)*cos(lat2r)*sin((lon2r-lon1r)/2)^2 ) );

    if delta < 1e-12
        lat_out = lat1; lon_out = lon1; 
        return;
    end

    % fraction of the great-circle arc to step
    f = ds_probe / (R*delta);  % Note: can be negative

    % slerp coefficients
    A = sin((1 - f)*delta) / sin(delta);
    B = sin(f*delta)        / sin(delta);

    % interpolate on the unit sphere
    x = A*cos(lat1r)*cos(lon1r) + B*cos(lat2r)*cos(lon2r);
    y = A*cos(lat1r)*sin(lon1r) + B*cos(lat2r)*sin(lon2r);
    z = A*sin(lat1r)              + B*sin(lat2r);

    lat_out = rad2deg( atan2(z, hypot(x,y)) );
    lon_out = rad2deg( atan2(y, x) );

    % normalize lon to [-180,180]
    lon_out = mod(lon_out + 180, 360) - 180;
end
