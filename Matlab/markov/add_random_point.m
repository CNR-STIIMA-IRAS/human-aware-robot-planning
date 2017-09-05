function rtraj = add_random_point( traj, n_rand, r_rand )

%     n_rand = 100;
%     r_rand = 0.1;
    rtraj  = zeros( length( traj ) * n_rand, 3 );
    for k=1:length( traj )
        rtraj( 1 + (k-1)*n_rand : k*n_rand, : )= repmat( traj(k,:), n_rand, 1) + ( 2 * r_rand*rand( n_rand, 3 ) - r_rand );
    end

    rtraj  = [ traj; rtraj ];