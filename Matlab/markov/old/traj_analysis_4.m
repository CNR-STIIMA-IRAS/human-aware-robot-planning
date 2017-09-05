clear all
close all
clc

addpath('VChoosek')

%%
% idx, x, y, z, occurrency
% oc: numero di occorrenze entando nella cella una sola volta

ds     = 0.01;
%%%%%%%%%%%%%%%%%%
HumanTaskRepetitions = 10;
Tstop_robot = 1.0;
Thuman      = 80.0 / HumanTaskRepetitions;
n_frame     = round( 2199 / HumanTaskRepetitions );
points      = load('C:\\Users\\NicolaPedrocchi\\OneDrive - C.N.R. ITIA\\Progetti\\shared-vb\\Sperim_Letters\\points.mat', '-ascii');
%%%%%%%%%%%%%%%%%%
id=points(:,1);
h_x=points(:,2);
h_y=points(:,3);
h_z=points(:,4);
h_oc=points(:,5) / HumanTaskRepetitions;
n_points = length(h_x);

%%
max_trial = 10;

BinCount  = zeros( max_trial, 1 );
Tmax      = zeros( max_trial, 1 );
Tmean     = zeros( max_trial, 1 );
Tvar      = zeros( max_trial, 1 );
Ncollision= zeros( max_trial, 1 );




for oo=1:max_trial
    
    ext_x = -0.5 + rand();
    ext_y = -0.5 + rand();
    ext_z = -0.5 + rand();
    %trajecory definition
    traj_x= linspace(min(h_x)-ext_x, max(h_x)+ext_x, 100)';
    traj_y= linspace(min(h_y)-ext_y, max(h_y)+ext_y, 100)';
    traj_z= linspace(min(h_z)-ext_z, max(h_z)+ext_z, 100)';

    traj_l = sum( sqrt( diff(traj_x).^2 + diff(traj_x).^2 + diff(traj_z).^2 ) );
    v_mean = 0.250;
    Tnom   = traj_l / v_mean;

    %%
    pts        =[h_x' traj_x'; h_y' traj_y'; h_z' traj_z']';
    [OG,TRAJ]  = octree_calc( pts, 1:length(h_x) ,length(h_x):length(pts), h_oc );

    %%
    % Markov Chain Properties
    combinations    = cell( OG.BinCount, 1 );
    n_combinations  = zeros( OG.BinCount, 1 );
    
    mc_mean = {};
    TexpectedMean = [];
    max_time_achieved = 0;
    n_collision = 0;
    for kAllowedCollision=1:OG.BinCount
        %%%%%%%%%%%%%%%%%%%%%%%%%%
        combinations{kAllowedCollision}   = VChooseK( 1:OG.BinCount, kAllowedCollision );
        n_combinations(kAllowedCollision) = size( combinations{kAllowedCollision}, 1 )';
         disp([num2str(oo), '/', num2str(max_trial), '# ', num2str(kAllowedCollision), '/', num2str(OG.BinCount), ' - Bin Evaluation, Number of possible combinations: ', num2str( n_combinations(kAllowedCollision) )]);

        %%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%
        TexpectedMeanComb = [];
        for iC = 1:n_combinations(kAllowedCollision)
            comb = combinations{kAllowedCollision}(iC,:);
            Texpected = MC( OG.BinCount, OG.BinMean, TRAJ.BinNSamples, comb, Thuman, Tstop_robot, Tnom, n_frame  );
            if Texpected <= Tnom + Thuman
                TexpectedMeanComb = [ TexpectedMeanComb ; Texpected ];
            end
        end
        if size( TexpectedMeanComb, 1 ) > 0
            n_collision   = kAllowedCollision;
            TexpectedMean = [ TexpectedMean; mean( TexpectedMeanComb ) ];
        else
            disp([num2str(oo), '/', num2str(max_trial), '#     *** Max time Achieved, number of collision: ',num2str( n_collision )])
            break;
        end
        
        if kAllowedCollision == OG.BinCount
            disp([num2str(oo), '/', num2str(max_trial), '#     *** Number of collision equal to the number of bins: ',num2str( n_collision )])
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
    if size( TexpectedMean, 1 ) > 0
        BinCount(  oo )  = OG.BinCount;
        Tmean( oo )      = mean( TexpectedMean );
        Tvar ( oo )      = std ( TexpectedMean );
        Tmax ( oo )      = Tnom + Thuman;
        Ncollision ( oo )= n_collision;
        disp([num2str(oo), '/', num2str(max_trial), '# T max      :',num2str(Tmax (oo)     )])
        disp([num2str(oo), '/', num2str(max_trial), '# T expected :',num2str(Tmean(oo)     )])
        disp([num2str(oo), '/', num2str(max_trial), '# T std      :',num2str(Tvar (oo)     )])
        disp([num2str(oo), '/', num2str(max_trial), '# N collision:',num2str(Ncollision(oo))])
    else
        disp('Error....');
    end
    
end





