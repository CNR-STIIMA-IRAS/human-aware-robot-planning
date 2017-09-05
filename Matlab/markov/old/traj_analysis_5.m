clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% INPUT
v_mean               = 0.250; % Mean Velocity along path
HumanTaskRepetitions = 10;
Tstop_robot          = 1.0;
Thuman               = 80.0 / HumanTaskRepetitions;
n_frame              = round( 2199 / HumanTaskRepetitions );
og                   = load('C:\\Users\\NicolaPedrocchi\\OneDrive - C.N.R. ITIA\\Progetti\\shared-vb\\Sperim_Letters\\points.mat', '-ascii');
trajs = { 'C:\\Users\\NicolaPedrocchi\\OneDrive - C.N.R. ITIA\\Progetti\\shared-vb\\Sperim_Letters\\traj_torun\\run2\\lbr_iiwa_14_r820_user1_traj17_1_matlab2.txt' }; %...
%         , 'C:\\Users\\NicolaPedrocchi\\OneDrive - C.N.R. ITIA\\Progetti\\shared-vb\\Sperim_Letters\\traj_torun\\run3\\lbr_iiwa_14_r820_user1_traj17_1_matlab2.txt' ...
%         , 'C:\\Users\\NicolaPedrocchi\\OneDrive - C.N.R. ITIA\\Progetti\\shared-vb\\Sperim_Letters\\traj_torun\\run4\\lbr_iiwa_14_r820_user1_traj17_1_matlab2.txt' ...
%         , 'C:\\Users\\NicolaPedrocchi\\OneDrive - C.N.R. ITIA\\Progetti\\shared-vb\\Sperim_Letters\\traj_torun\\run5\\lbr_iiwa_14_r820_user1_traj17_1_matlab2.txt' 
%};   

og_points = zeros( size(og,1), 3 );
og_points(:,1) = -og(:,3);
og_points(:,2) = og(:,2);
og_points(:,3) = og(:,4);
og_occurences  = ceil( og(:,5) / HumanTaskRepetitions );
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% include the compiled library for the definition of the coefficients of
%%% choosek, i.e. the vector of all the combination given by the binomial
%%% formula
%%% ( n )      n!
%%% (   ) =  ---------
%%% ( k )    k! (n-k)!
%%% The choosek coefficients are all the combinations
addpath('VChoosek')


%%
%%% For each different experiment
for oo=1:length( trajs )

    disp('=========================================================');
    disp(trajs{oo})
    disp('=========================================================');
    [ robot_traj, Tnom ]  = robot_trajectory( trajs{oo}, v_mean );

    %%% Clustering of HUMAN GRID and TRAJECTORY using the OCTREE
    % OG: OCTREE describing the HUMAN Grid after the clustering
    % TRAJ: OCTREE describing the HUMAN Grid after the clustering
    %     NOTE: OG and TRAJ are given by the intersection of the human
    %     swept volume and trajectory volume. Therefore, in a generic bin
    %     (OG.Bin or TRAJ.BIN) there is at least one point belonging to the
    %     human volume and one belonging to the TRAJ volume
    [OG,TRAJ]  = octree_calc( [ og_points; robot_traj ] ...                 % Concatenate the GRID points and the TRAJECTORY points
                            , 1:length(og_points) ...                       % index of the grid points
                            , length(og_points) + (index_trajcomplete)...   % index of the trajecotry points (robot as a cloud) 
                            , 50  ...                                       % decimation factor of the OCTREE 
                            , og_occurences ...                             % occurencies, i.e., numer of instant times at least one point of the person crossed the corresponding grid point
                            , 1 ...                                         % show plot or not
                            );
    
    %%%% here for efficiency
    combinations    = cell( OG.BinCount, 1 );       
    n_combinations  = zeros( OG.BinCount, 1 );
    
    lDiag = logical( eye ( OG.BinCount, OG.BinCount ) );
    uDiag = logical( triu( ones(OG.BinCount,OG.BinCount ), 1 ) - triu( ones(OG.BinCount,OG.BinCount), 2 ) );
    mc_mean = {};
    TexpectedMean = [];
    max_time_achieved = 0;
    n_collision = 0;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
    %%% Assumptions:
    %%% 1) The HUMAN OCCUPANCY GRID may have different nodes with the
    %%% HUMAN OCCUPANCY RATE is equal to 100%
    %%%
    %%% 2) Except that in the last point of the trajectory, the ROBOT does
    %%% not cross any point of the HUMAN OCCUPANCY GRID with an HUMAN
    %%% OCCUPANCY RATE euqal to 100%
    %%%
    %%% 3) All the Bins are mainly of the same size
    %%%
    %%% 4) Only one collision can happen for each bin: if the robot
    %%% collides in a Bin, the next collisions may happen in the next Bins,
    %%% and it cannot happen again in the same Bin.
    %%% 
    %%% 5) The maximum number of collision is equal to the maximum number
    %%% of Bins
    nAllowedCollision = OG.BinCount;
    %%%
    %%% 6) The probability the robot gets a collision in the Bin is
    %%% described by the HUMAN OCCUPANCY RATE (e.g., the mean number of
    %%% occurences in the Bin divided by the number of the experiments
    %%% frames)
    %%% 
    %%% 7) The robot can moves only from a Bin to the sequent Bin; 
    
    for kAllowedCollision=1:nAllowedCollision
        %%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% include the compiled library for the definition of the coefficients of
        %%% choosek, i.e. the vector of all the combination given by the binomial
        %%% formula
        %%% ( n )      n!
        %%% (   ) =  ---------
        %%% ( k )    k! (n-k)!
        %%% The choosek coefficients are all the combinations
        combinations{kAllowedCollision}   = VChooseK( 1:OG.BinCount, kAllowedCollision );
        n_combinations(kAllowedCollision) = size( combinations{kAllowedCollision}, 1 )';
        disp([num2str(kAllowedCollision), '/', num2str(OG.BinCount), ' - Bin Evaluation, Number of possible combinations: ', num2str( n_combinations(kAllowedCollision) )]);
        %%%%%%%%%%%%%%%%%%%%%%%%%%

        %%%%%%%%%%%%%%%%%%%%%%%%%%
        TexpectedMeanComb = zeros( n_combinations(kAllowedCollision), 1 );
        iNonNull          = zeros( n_combinations(kAllowedCollision), 1 );
        for iC = 1:n_combinations(kAllowedCollision)
            comb = combinations{kAllowedCollision}(iC,:);
            Texpected = MC( OG.BinCount, OG.BinMean, TRAJ.BinNSamples, comb, Thuman, Tstop_robot, Tnom, n_frame, lDiag, uDiag );
            if Texpected <= Tnom + Thuman
                TexpectedMeanComb( iC ) = Texpected;
                iNonNull( iC ) = 1;
            end
        end
        if size( TexpectedMeanComb(logical(iNonNull)), 1 ) > 0
            n_collision   = kAllowedCollision;
            TexpectedMean = [ TexpectedMean; mean( TexpectedMeanComb(logical(iNonNull)) ) ];
        else
            disp(['     *** Max time Achieved, number of collision: ',num2str( n_collision )])
            break;
        end

        if kAllowedCollision == OG.BinCount
            disp(['     *** Number of collision equal to the number of bins: ',num2str( n_collision )])
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%
    end

    if size( TexpectedMean, 1 ) > 0
        BinCount  = OG.BinCount;
        Tmean      = mean( TexpectedMean );
        Tvar       = std ( TexpectedMean );
        Tmax       = Tnom + Thuman;
        Ncollision = n_collision;
        disp('----------------')
        disp([' T nom      :',num2str(Tnom      )])
        disp([' T max      :',num2str(Tmax      )])
        disp('----------------')
        disp([' DT expected:',num2str(Tmean - Tnom )])
        disp([' T expected :',num2str(Tmean     )])
        disp([' T std      :',num2str(Tvar      )])
        disp('----------------')
        disp([' N collision:',num2str(Ncollision)])
        disp('----------------')
    else
        disp('Error....');
    end


end



