function [ Tmean, Tvar, Tmax, NCmax, NCmin, NCmean, MCMax, MCEff, BinCount ] = markov_traj_analysis( og_points, og_occurences, robot_traj, Tstop_robot, Thuman, Tnom, n_rand, r_rand, r_bb )

%%% Clustering of HUMAN GRID and TRAJECTORY using the OCTREE
% OG: OCTREE describing the HUMAN Grid after the clustering
% TRAJ: OCTREE describing the HUMAN Grid after the clustering
%     NOTE: OG and TRAJ are given by the intersection of the human
%     swept volume and trajectory volume. Therefore, in a generic bin
%     (OG.Bin or TRAJ.BIN) there is at least one point belonging to the
%     human volume and one belonging to the TRAJ volume
[TrajPointsBin, TrajPoints, TrajBinOrder, TrajBinNElements, HocBinMean, HocBinSum, HocBinMax, HocBinMin ] = octree_calc( og_points, robot_traj, 50, og_occurences, n_rand, r_rand,r_bb, 0 );

BinCount = length( TrajBinOrder );
%%%% here for efficiency
combinations    = cell( BinCount, 1 );       
n_combinations  = zeros( BinCount, 1 );

lDiag = logical( eye ( BinCount, BinCount ) );
uDiag = logical( triu( ones(BinCount,BinCount ), 1 ) - triu( ones(BinCount,BinCount), 2 ) );
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
nAllowedCollision = BinCount;
%%%
%%% 6) The probability the robot gets a collision in the Bin is
%%% described by the HUMAN OCCUPANCY RATE (e.g., the mean number of
%%% occurences in the Bin divided by the number of the experiments
%%% frames)
%%% 
%%% 7) The robot can moves only from a Bin to the sequent Bin; 

coll_freq = zeros(1,nAllowedCollision);
MCEff = 0;
for kAllowedCollision=1:nAllowedCollision
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% include the compiled library for the definition of the coefficients of
    %%% choosek, i.e. the vector of all the combination given by the binomial
    %%% formula
    %%% ( n )      n!
    %%% (   ) =  ---------
    %%% ( k )    k! (n-k)!
    %%% The choosek coefficients are all the combinations
    combinations{kAllowedCollision}   = VChooseK( 1:BinCount, kAllowedCollision );
    n_combinations(kAllowedCollision) = size( combinations{kAllowedCollision}, 1 )';
    disp([num2str(kAllowedCollision), '/', num2str(BinCount), ' - Bin Evaluation, Number of possible combinations: ', num2str( n_combinations(kAllowedCollision) )]);
    %MCMax = MCMax + ( n_combinations(kAllowedCollision) );
    %%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    TexpectedMeanComb = zeros( n_combinations(kAllowedCollision), 1 );
    iNonNull          = zeros( n_combinations(kAllowedCollision), 1 );
    for iC = 1:n_combinations(kAllowedCollision)
        comb = combinations{kAllowedCollision}(iC,:);
        Texpected = MC( BinCount, HocBinMean, TrajBinNElements, comb, Thuman, Tstop_robot, Tnom, lDiag, uDiag );
        if Texpected <= Tnom + Thuman
            TexpectedMeanComb( iC ) = Texpected;
            iNonNull( iC ) = 1;
            coll_freq(kAllowedCollision) = coll_freq(kAllowedCollision) + 1;
            MCEff = MCEff + 1;
        end        
    end
    if size( TexpectedMeanComb(logical(iNonNull)), 1 ) > 0
        n_collision   = kAllowedCollision;
        TexpectedMean = [ TexpectedMean; mean( TexpectedMeanComb(logical(iNonNull)) ) ];
    else
        disp(['     *** Max time Achieved, number of collision: ',num2str( n_collision )])
        break;
    end

    if kAllowedCollision == BinCount
        disp(['     *** Number of collision equal to the number of bins: ',num2str( n_collision )])
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%
end
MCMax = sum(n_combinations);

if size( TexpectedMean, 1 ) > 0
    Tmean      = mean( TexpectedMean );
    Tvar       = std ( TexpectedMean );
    Tmax       = Tnom + Thuman;
    NCmax = n_collision;
    NCmin = 1;
    NCmean = sum(coll_freq.*[1:1:length(coll_freq)])/sum(coll_freq);
else
    Tmean = Tnom;
    Tvar = 0;
    Tmax = Tnom;
    NCmax = 0;
    NCmin = 0;
    NCmean = 0;
end

