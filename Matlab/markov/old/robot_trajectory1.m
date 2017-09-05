function [ robot_traj, Tnom, index_trajcomplete, index_trajorig, index_trajrand ] = robot_trajectory( file_name, v_mean )
    % Load the trajectory point
    data  = load( file_name );
    nn   = [ 0; cumsum( sqrt( diff(data(:,1)).^2 + diff(data(:,2)).^2 + diff(data(:,3)).^2) ) ];
    rr   = linspace( 0, nn(end), 100 );
    robot_traj =  [ ppval( rr, spline( nn, data(:,1) ) )' ... 
                   , ppval( rr, spline( nn, data(:,2) ) )' ...
                   , ppval( rr, spline( nn, data(:,3) ) )' ];

    Tnom   = nn(end) / v_mean;

    n_rand = 100;
    r_rand = 0.1;
    rtraj  = zeros( length( traj ) * n_rand, 3 );
    for k=1:length( traj )
        rtraj( 1 + (k-1)*n_rand : k*n_rand, : )= repmat( traj(k,:), n_rand, 1) + ( 2 * r_rand*rand( n_rand, 3 ) - r_rand );
    end

    robot_traj         = [ traj; rtraj ];
    index_trajorig     = 1:length( traj );
    index_trajcomplete = 1:length( robot_traj );
    index_trajrand     = (length( traj )+1):length( robot_traj );
