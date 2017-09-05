function [ robot_traj, Tnom ] = robot_trajectory( file_name, v_mean )
    % Load the trajectory point
    data  = load( file_name );
    nn   = [ 0; cumsum( sqrt( diff(data(:,1)).^2 + diff(data(:,2)).^2 + diff(data(:,3)).^2) ) ];
    rr   = linspace( 0, nn(end), 100 );
    robot_traj =  [ ppval( rr, spline( nn, data(:,1) ) )' ... 
                   , ppval( rr, spline( nn, data(:,2) ) )' ...
                   , ppval( rr, spline( nn, data(:,3) ) )' ];

    Tnom   = nn(end) / v_mean;
