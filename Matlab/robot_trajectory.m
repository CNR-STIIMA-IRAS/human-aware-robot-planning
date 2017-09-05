function [ robot_traj, rr, Tnom ] = robot_trajectory( file_yaml_name, robot, n_wrist_joint, n_elbow_joint )
    % Load the trajectory point
    YamlStruct = ReadYaml(file_yaml_name);
    
    joint_positions = zeros( size( YamlStruct.euroc_traj.joint_positions ) );
    for i=1:size(joint_positions,1)
        for j=1:size(joint_positions,2)
            joint_positions(i,j) = YamlStruct.euroc_traj.joint_positions{i,j};
        end
    end
    
    %%% to do forward kin
     %%%% from joint_positions to 
     %%%% data(:,1:3) = end-effector; 
     %%%% data(:,4:6) = wrist
     %%%% data(:,7:9) = elbow
    %%%% 
    data  = 100 * ones( size( joint_positions, 1 ), 9 );
    for i=1:size(joint_positions,1)
        %end-effector
        temp = robot.fkine(joint_positions(i,:));
        data(i,1:3) = temp(1:3,4);
        %wrist      
        temp = robot.base()*robot.A([1:n_wrist_joint],joint_positions(i,:));
        data(i,4:6) = temp(1:3,4);
        %elbow 
        temp = robot.base()*robot.A([1:n_elbow_joint],joint_positions(i,:));
        data(i,7:9) = temp(1:3,4);        
    end
%     figure()
%     plot3(data(:,1),data(:,2),data(:,3), '-r', ...
%           data(:,4),data(:,5),data(:,6),'-b', ...
%           data(:,7),data(:,8),data(:,9),'-g')
%     grid on 
%     legend('endeff', 'wrist', 'elbow');

    nn   = [ 0; cumsum( sqrt( diff(data(:,1)).^2 + diff(data(:,2)).^2 + diff(data(:,3)).^2) ) ];
    rr   = linspace( 0, nn(end), 100 );
    robot_traj =   [ ppval( rr, spline( nn, data(:,1) ) )' ... 
                   , ppval( rr, spline( nn, data(:,2) ) )' ...
                   , ppval( rr, spline( nn, data(:,3) ) )' ...
                   , ppval( rr, spline( nn, data(:,4) ) )' ... 
                   , ppval( rr, spline( nn, data(:,5) ) )' ...
                   , ppval( rr, spline( nn, data(:,6) ) )' ...
                   , ppval( rr, spline( nn, data(:,7) ) )' ... 
                   , ppval( rr, spline( nn, data(:,8) ) )' ...
                   , ppval( rr, spline( nn, data(:,9) ) )' ...
                   ];

    Tnom = YamlStruct.euroc_traj.time;
