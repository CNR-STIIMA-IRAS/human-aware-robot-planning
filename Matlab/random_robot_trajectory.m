function [ robot_traj, s, Tnom ] = random_robot_trajectory( robot, n_wrist_joint, n_elbow_joint, bb, v_mean, acc, dec, what )

nviapoint  = 100;
robot_traj = 100 * ones( nviapoint, 9 );

if strcmp(what,'cart') 
    startpoint = [ bb(1); bb(3); bb(5)] + 0.2 * diag( rand(3,1) ) * (  [ bb(2); bb(4); bb(6)] - [ bb(1); bb(3); bb(5) ] );
    endpoint   = [ bb(2); bb(4); bb(6)] + 0.2 * diag( rand(3,1) ) * (  [ bb(1); bb(3); bb(5)] - [ bb(2); bb(4); bb(6) ] );
    uv         = ( endpoint - startpoint ) / norm( endpoint - startpoint );
    endpoint   = startpoint + (0.8 + 0.4 * rand() ) * uv;
    dd         = endpoint - startpoint;
    step       = dd / nviapoint;
    vv         = dd / norm(dd);
    for ipnt = 1:nviapoint
        robot_traj(ipnt, 1:3) = startpoint + (ipnt-1)/(nviapoint-1) * vv;
        robot_traj(ipnt, 1:3) = robot_traj(ipnt, 1:3) +  step' * diag( rand(3,1) );
    end
end

if strcmp(what,'joint')  
    
    max_reacheable_dist = 1.4;
    min_reacheable_dist = 0.2;
    
    bbv = [bb(1) bb(3) bb(5);
           bb(2) bb(3) bb(5);
           bb(1) bb(4) bb(5);
           bb(2) bb(4) bb(5);
           bb(1) bb(3) bb(6);
           bb(2) bb(3) bb(6);
           bb(1) bb(4) bb(6);
           bb(2) bb(4) bb(6) ]; 
    OG        = mean( bbv );
    RB        = robot.base(1:3,4);
    bbv_dist  = sqrt(( bbv(:,1)-RB(1)).^2+(bbv(:,2)-RB(2)).^2+(bbv(:,3)-RB(3)).^2);
    [ bbv_dist_sorted, I ] = sort( bbv_dist );
    bbv_sorted = bbv( I, : );
    
    idx_bbv_reacheable = find( bbv_dist_sorted < max_reacheable_dist );
    bbv_reacheable = bbv_sorted( idx_bbv_reacheable, :);
    if isempty( bbv_reacheable )
        error('The robot seems always out the oggupancy grid');
    end
    
    qq = [ atan2( bbv_reacheable(:,2), bbv_reacheable(:,1) ) , 1.+rand(size(bbv_reacheable,1), robot.n-1 ) ] ;
    xx_reacheable = [];
    for i=1:size(qq,1)
        xp = robot.fkine( qq(i,:) );
        xm = robot.fkine( [ qq(i,1), -qq(i,2:end) ] );
        if norm( xp(1:3,4) - bbv_reacheable(i,:)' ) < norm( xm(1:3,4) - bbv_reacheable(i,:)' )
            xx_reacheable = [ xx_reacheable; xp(1:3,4)' ];
        else
            xx_reacheable = [ xx_reacheable; xm(1:3,4)' ];
        end
    end
    
    
    T =eye(4);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    idx_start = 1;
    if  size( xx_reacheable, 1 ) > 2  
        idx_start = randi( size( xx_reacheable, 1 ), 1 );
    end
    startpoint = xx_reacheable( idx_start, : )';
    T(1:3,1:3)=unitarand(3,3); if det(T(1:3,1:3)) < 0, T(1:3,1:3) = -T(1:3,1:3); end
    T(1:3,4)  =startpoint;
    q = ikine6sMOD(robot,T,zeros(6,1));  
    counter = 0;
    while isempty(q) && counter<10000
        T(1:3,1:3)=unitarand(3,3); if det(T(1:3,1:3)) < 0, T(1:3,1:3) = -T(1:3,1:3); end
        q = ikine6sMOD(robot,T,zeros(6,1));
        counter = counter + 1;
    end
    if counter>=10000
        error([ '#startpoint counter > 10000: random orientation not identified'])
    end
    q_points(1,:)=q(1,:);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    viapoint   = OG' + 0.1 * rand(3,1) ;
    if norm( viapoint - RB ) > max_reacheable_dist
        q_points(2,1) = atan2(viapoint(2)-RB(2),viapoint(1)-RB(1)) + (-0.1+rand()*0.1);
        q_points(2,2) = atan2(viapoint(3)-RB(3),sqrt((viapoint(1)-RB(1))^2+(viapoint(2)-RB(2))^2) ) + (-0.1+rand()*0.1);
        q_points(2,3) = (-0.1+rand()*0.1);
        q_points(2,4) = (-0.1+rand()*0.1);
        q_points(2,5) = (-0.1+rand()*0.1);
        q_points(2,6) = (-0.1+rand()*0.1);
    else
        T(1:3,1:3)=unitarand(3,3); if det(T(1:3,1:3)) < 0, T(1:3,1:3) = -T(1:3,1:3); end
        T(1:3,4)  =viapoint;
        q = ikine6sMOD(robot,T,q_points(1,:)');  
        counter = 0;
        while isempty(q) && counter<10000
            T(1:3,1:3)=unitarand(3,3); if det(T(1:3,1:3)) < 0, T(1:3,1:3) = -T(1:3,1:3); end
            q = ikine6sMOD(robot,T,q_points(1,:)');
            counter = counter + 1;
        end
        if counter>=10000
            error([ '#viapoint counter > 10000: random orientation not identified'])
        end
        q_points(2,:)=q(1,:);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    endpoint   = bbv_sorted(end,:)' ;
    if norm( endpoint - RB ) > max_reacheable_dist
        q_points(3,1) = atan2(viapoint(2)-RB(2),viapoint(1)-RB(1)) + (-0.1+rand()*0.1);
        q_points(3,2) = atan2(viapoint(3)-RB(3),sqrt((viapoint(1)-RB(1))^2+(viapoint(2)-RB(2))^2) ) + (-1+rand()*2);
        q_points(3,3) = (-1+rand()*2);
        q_points(3,4) = (-1+rand()*2);
        q_points(3,5) = (-1+rand()*2);
        q_points(3,6) = (-1+rand()*2);
    else
        T(1:3,1:3)=unitarand(3,3); if det(T(1:3,1:3)) < 0, T(1:3,1:3) = -T(1:3,1:3); end
        T(1:3,4)  =endpoint;
        q = ikine6sMOD(robot,T,q_points(2,:)');  
        counter = 0;
        while isempty(q) && counter<10000
            T(1:3,1:3)=unitarand(3,3); if det(T(1:3,1:3)) < 0, T(1:3,1:3) = -T(1:3,1:3); end
            q = ikine6sMOD(robot,T,q_points(2,:)');
            counter = counter + 1;
        end
        if counter>=10000
            error([ '#endpoint counter > 10000: random orientation not identified'])
        end
        q_points(3,:)=q(1,:);
    end        
    
    jdist = zeros( size(q_points,1)-1, 1);
    for i=1:robot.n
        jdist = jdist + diff(q_points(:,i)).^2;
    end
    jdist = sqrt( jdist );
    nn    = [ 0; cumsum( jdist ) ];
    rr    = linspace( 0, nn(end), 100 );
    q_robot_traj = zeros(length(rr),robot.n);
    for i=1:robot.n
        q_robot_traj(:,i) =  ppval( rr, spline( nn, q_points(:,i) ) );
    end
    
    robot_traj  = ones( size(q_robot_traj,1), 9 );
    for i=1:size(q_robot_traj,1)              
        temp = robot.fkine(q_robot_traj(i,:));
        robot_traj(i,1:3)= temp(1:3,4)';
        %wrist      
        temp = robot.base()*robot.A([1:n_wrist_joint],q_robot_traj(i,:));
        robot_traj(i,4:6) = temp(1:3,4)';
        %elbow 
        temp = robot.base()*robot.A([1:n_elbow_joint],q_robot_traj(i,:));
        robot_traj(i,7:9) = temp(1:3,4)'; 
    end   
end

% figure()
% plot3(robot_traj(:,1),robot_traj(:,2),robot_traj(:,3), '-r', ...
%       robot_traj(:,4),robot_traj(:,5),robot_traj(:,6),'-b', ...
%       robot_traj(:,7),robot_traj(:,8),robot_traj(:,9),'-g')
% grid on 
% legend('endeff', 'wrist', 'elbow');

s    = [ 0; cumsum( sqrt( diff(robot_traj(:,1)).^2 + diff(robot_traj(:,2)).^2 + diff(robot_traj(:,3)).^2) ) ];
t0   = v_mean / acc;
t1   = v_mean / dec;
Tnom = t0 + t1 + ( s(end) - 0.5*acc*t0^2 - 0.5*acc*t1^2 ) / v_mean;
