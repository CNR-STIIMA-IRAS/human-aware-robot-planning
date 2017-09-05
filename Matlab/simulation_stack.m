if 0
    clear all;
    close all;
    clc;

    
    addpath('.\\VChooseK');
    addpath('.\\markov'); 
    addpath('.\\trajgen');
    addpath('.\\YAMLMatlab');
    addpath('.\\rvctools');

    %%%%% Human data
    disp('----------------------------------------------------');
    disp('Load Human Data')
    disp('----------------------------------------------------');
    HumanTaskRepetitions = 9;
    [ kinect_data, THumanSingleTask ] = prepare_workspace_to_simulation( 'data\march2017\exp1', HumanTaskRepetitions );
    %[ kinect_data, THumanSingleTask ] = prepare_workspace_to_simulation( 'data\September2016\meshes\file_for_matlab', HumanTaskRepetitions );
    

    figure; 
    for iFrame =1:kinect_data.xvalues.signals.dimensions
        hold on;
        plot3( kinect_data.xvalues.signals.values(:,iFrame), kinect_data.yvalues.signals.values(:,iFrame), kinect_data.zvalues.signals.values(:,iFrame) );
        grid on;
        axis equal;
    end    
end

close all
clear simulation markov robot L

run_sim=1; %to run simulation

%%%%%%%%%%%%%%%% Config
    %%%%% Robot
    
    calib_sim = 1;
    
    
    %%%%% Trajectory data    
    robot_random_traj = 1;  %1 for random trajecory; 0 for the analysis of an existing traj
    
    if robot_random_traj == 0
        kuka_iiwa14;
        calib_sim         = 0;  %to define velocity and speed in simulink for trajecory whose Tnom is given
        traj_files = {'data\\September2016\\traj_torun\\run2\\lbr_iiwa_14_r820_user1_traj17_1_exec.yaml';
                      'data\\September2016\\traj_torun\\run3\\lbr_iiwa_14_r820_user1_traj17_1_exec.yaml';
                      'data\\September2016\\traj_torun\\run4\\lbr_iiwa_14_r820_user1_traj17_1_exec.yaml';
                      'data\\September2016\\traj_torun\\run5\\lbr_iiwa_14_r820_user1_traj17_1_exec.yaml'};
        %run2: 0.1 0.250
                  
        nStack         = size(traj_files,1);
        if calib_sim == 1
            min_dist_2   = -0.250;            
            nTrial         = 1;
        else
            min_dist_2   = 0.250;
            nTrial         = 100;
        end
    else
        kuka_iiwa14_6dof;
        nStack = 30;
        nTrial = 60;
        min_dist_2 = 0.250;
    end
    n_rand            = 500; %number of random points
    r_bb             = 0.05; %robot radius

    %%%%% Trajectory data
    acc            = 2.0; 
    dec            = 2.0; 
    v_mean         = 0.250; % Mean Velocity along path
    Tstop_robot    = v_mean / acc;
%%%%%%%%%%%%%%%% End Config


%%%%%%%%%%%%%%%% Prepare simulation and markov
dt_simulation = 0.05;
bb = [ min( kinect_data.og_points(:,1) ), max( kinect_data.og_points(:,1) ) ...
     , min( kinect_data.og_points(:,2) ), max( kinect_data.og_points(:,2) ) ...
     , min( kinect_data.og_points(:,3) ), max( kinect_data.og_points(:,3) ) ];
    
simuation.Tmean = zeros(nStack,1);
simuation.Tmin  = zeros(nStack,1);
simuation.Tmax  = zeros(nStack,1);
simuation.Tstd  = zeros(nStack,1);
simuation.NCmean  = zeros(nStack,1);
simuation.NCmin  = zeros(nStack,1);
simuation.NCmax  = zeros(nStack,1);

markov.Tmean = zeros(nStack,1);
markov.Tmax  = zeros(nStack,1);
markov.Tstd  = zeros(nStack,1);
markov.NCmax  = zeros(nStack,1);
markov.NCmin  = zeros(nStack,1);
markov.NCmean  = zeros(nStack,1);

robottrajs = cell(1,nStack);

mkchain_numbers.TotChains = zeros(nStack,1);
mkchain_numbers.UsedChains = zeros(nStack,1);
%%%%%%%%%%%%%%%% End Prepare simulation and markov

for iStack = 1:nStack

    disp('----------------------------------------------------');
    disp([ num2str(iStack),'/',num2str(nStack) ] );
    disp('----------------------------------------------------');
    
    clear robot_trajectory_data robot_trajectory_s Tnom 
    
    if robot_random_traj == 1
        disp('Generate Trajectory')
        %[ robot_trajectory_data, robot_trajectory_s, Tnom ] = random_robot_trajectory( bb, v_mean, acc, dec, 'cart' );   
        [ robot_trajectory_data, robot_trajectory_s, Tnom ] = random_robot_trajectory( robot, robot.n-1, robot.n-3, bb, v_mean, acc, dec, 'cart' );   
        min_dist = -0.1;
        start_time = 0.0;
        [t, state, out] = sim ( 'trajgen2_ste2_realtraj',[ kinect_data.xvalues.time(1) ceil( (kinect_data.xvalues.time(1) + THumanSingleTask  + 2* Tnom ) / dt_simulation ) * dt_simulation ] );
        jerk = diff(abs( out(:,4) ));
        idx_robot_movement = find( abs( out(:,4) ) > 0 );
        Tnom = t(idx_robot_movement(end)) - t( idx_robot_movement(1) );
    else
        disp('Load Trajectory')
        [ robot_trajectory_data, robot_trajectory_s, Tnom ] = robot_trajectory (  traj_files{iStack}, robot, robot.n-1, robot.n-3 );
%         loc_x = robot_trajectory_data(:,1);
%         loc_y = robot_trajectory_data(:,2);
%         robot_trajectory_data(:,1)=loc_y;
%         robot_trajectory_data(:,2)=-loc_x;
    end
    
    min_dist = min_dist_2;
    robottrajs{iStack} = robot_trajectory_data;
    
    Tmean_ = []; 
    Ncoll_ = zeros(1,nTrial);
    
    if run_sim==1
        disp('Simulate Trajectory')
        for iTrial=1:nTrial
            start_time = kinect_data.xvalues.time(1) + ( THumanSingleTask ) * rand();
            [t, state, out] = sim ( 'trajgen2_ste2_realtraj',[ kinect_data.xvalues.time(1) ceil( (kinect_data.xvalues.time(1) + THumanSingleTask  + 2* Tnom ) / dt_simulation ) * dt_simulation ] );
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%% debug
    %         figure(1000*iStack+1); 
    %         plot(t, out(:,1), t, out(:,2), t, out(:,5),t, out(:,6),t, out(:,7), t, min_dist * ( ones( size( t,1 ) ) ) );
    %         legend('s', 'Delayed target', 'human-robot dist', 'Nominal target', 'Actual target', 'critical dist');
    %         figure(100*iStack); 
    %         plot( t, out(:,1) );
    %         legend('s');
    %         hold on; 
    %         figure(2000*iStack+1); 
            plot( t, out(:,1), t, out(:,5),t, min_dist * ( ones( size( t,1 ) ) ) );
            legend('Robot trajectory','Human-robot distance','Critical distance');
    %         figure(3000*iStack+1); 
    %         plot( t, out(:,1), t, out(:,5) );
    %         legend('Robot trajectory','Human-robot distance');
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            jerk = diff(abs( out(:,4) ));
            idx_robot_movement = find( abs( out(:,4) ) > 0 );
            collisions =  out(:,5) < min_dist ;
            Ncoll_(iTrial) = 0;
            for k=1:length(collisions)-1
                if collisions(k)==1 && collisions(k+1)==0
                    Ncoll_(iTrial)=Ncoll_(iTrial)+1;
                end
            end
            %disp([ num2str(iTrial),'/',num2str(iStack),'# tstart: ', num2str(start_time), ' t1:', num2str(t(idx_robot_movement(1))), ' tend: ', num2str(t(idx_robot_movement(end))) ])
            Tmean_ = [Tmean_; t(idx_robot_movement(end)) - t( idx_robot_movement(1) ); ];
        end

        simulation.Tmean( iStack ) = mean( Tmean_ );
        simulation.Tstd ( iStack ) = std( Tmean_ );
        simulation.Tmax ( iStack ) = max( Tmean_ );
        simulation.NCmean( iStack ) = mean ( Ncoll_ );
        simulation.NCmin( iStack ) = min ( Ncoll_ );
        simulation.NCmax( iStack ) = max ( Ncoll_ );
        disp('----------------')
        disp([' T nom      :',num2str(Tnom      )])
        disp([' DT expected:',num2str(mean( Tmean_ ) - Tnom )])
        disp([' T expected :',num2str(mean( Tmean_ ))])
        disp([' T std      :',num2str(std( Tmean_ ))])
        disp([' T max      :',num2str(max( Tmean_ ))])
        disp([' N collision:',num2str(mean ( Ncoll_ ))])
        disp('----------------')
    end

    if 1
        disp([ num2str(iStack),'/',num2str(nStack),' Markov Estimation' ])

        [ markov.Tmean(iStack), markov.Tstd(iStack), markov.Tmax(iStack), ... 
          markov.NCmax(iStack),  markov.NCmin(iStack), markov.NCmean(iStack), ...
          mkchain_numbers.TotChains(iStack), mkchain_numbers.UsedChains(iStack), ...
          BinCount ] = markov_traj_analysis ( kinect_data.og_points ...
                                                , kinect_data.og_occurences ...
                                                , robot_trajectory_data(:,1:3) ...
                                                , Tstop_robot ...
                                                , THumanSingleTask ...
                                                , Tnom   ...                                                                                                               
                                                , n_rand ...
                                                , min_dist ...
                                                , r_bb   );
    

        disp([' T nom      :',num2str(Tnom      )])
        disp([' DT expected:',num2str(markov.Tmean(iStack) - Tnom )])
        disp([' T expected :',num2str(markov.Tmean(iStack)     )])
        disp([' T std      :',num2str(markov.Tstd(iStack)      )])
        disp([' T max      :',num2str(markov.Tmax(iStack)      )])
        disp([' N collision:',num2str(markov.NCmean(iStack))])
        disp('----------------')
    
        if run_sim==1
            disp([' error:',num2str(markov.Tmean(iStack)-simulation.Tmean(iStack))])
        end
    end
    save('stefania.mat');
end



% plot(1:iStack, markov.Tmean,'-r',1:iStack,markov.Tstd,'-',1:iStack,markov.Tmax,'-', ...
%      1:iStack, simulation.Tmean,'*',1:iStack,simulation.Tstd,'*',1:iStack,simulation.Tmax,'*')

if run_sim==1
    [~,I] = sort(simulation.Tstd);
    figure()
    plot(1:iStack, markov.Tmean(I),'-b', ...
         1:iStack, markov.Tmean(I) + markov.Tstd(I),'--b', ...
         1:iStack, markov.Tmean(I) - markov.Tstd(I),'--b', ...
         1:iStack, simulation.Tmax(I),'-r')
    title('')
    xlabel('Trajectories') % x-axis label
    ylabel('Time [s]') % y-axis label
    legend('Approch expected time','Approach standard deviation','Approach standard deviation','Simulation maximum time')


    figure()
    plot(1:iStack, markov.Tmean(I),'-b', ...
         1:iStack, markov.Tmax(I),'--b', ...
         1:iStack, simulation.Tmax(I),'-r')
    title('')
    xlabel('Trajectories') % x-axis label
    ylabel('Time [s]') % y-axis label
    legend('Approch expected time','Approach max time','Simulation maximum time')


    % figure()
    % histogram(simulation.NCmax);
    % hold on
    % histogram(markov.NCmax);
    % title('Collisions max')
    % xlabel('Number of robot stops') % x-axis label
    % ylabel('Frequency') % y-axis label
    % legend('Simulation','Approach')
    % 
    % figure()
    % histogram(simulation.NCmin);
    % hold on
    % histogram(markov.NCmin);
    % title('Collisions min')
    % xlabel('Number of robot stops') % x-axis label
    % ylabel('Frequency') % y-axis label
    % legend('Simulation','Approach')
    % 
    % figure()
    % histogram(round(simulation.NCmean));
    % hold on
    % histogram(round(markov.NCmean));
    % title('Collisions mean')
    % xlabel('Number of robot stops') % x-axis label
    % ylabel('Frequency') % y-axis label
    % legend('Simulation','Approach')

    figure()
    plot(1:iStack, simulation.Tmean(I),'-b', ...
         1:iStack, simulation.Tmean(I) + simulation.Tstd(I),'--b', ...
         1:iStack, simulation.Tmean(I) - simulation.Tstd(I),'--b', ...
         1:iStack, markov.Tmean(I),'-r')
    title('')
    xlabel('Trajectories') % x-axis label
    ylabel('Time [s]') % y-axis label
    legend('Simulation mean time','Simulation standard deviation','Simulation standard deviation','Approach mean time')

    figure()
    plot(1:1:iStack, markov.Tmean(I)- simulation.Tmean(I)')
    title('Error in expected robot time')
    xlabel('Trajectories') % x-axis label
    ylabel('Error in expected time estimation [s]') % y-axis label

end

figure()
plot(1:size(mkchain_numbers.UsedChains,1),mkchain_numbers.UsedChains,1:size(mkchain_numbers.TotChains,1),mkchain_numbers.TotChains)
%title('Error in expected robot time')
xlabel('Experiments') % x-axis label
ylabel('Number of Markov Chains') % y-axis label
legend('Used','Max')

plot_human_traj;


 
 