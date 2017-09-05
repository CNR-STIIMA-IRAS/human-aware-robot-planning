function [ TrajPointsBin, TrajPoints, TrajBinOrder, TrajBinNElements, HocBinMean, HocBinSum, HocBinMax, HocBinMin ] = octree_calc( og_pts ...
                                                                                                                                 , traj_pts ...
                                                                                                                                 , decimation_factor ...
                                                                                                                                 , h_oc ...
                                                                                                                                 , n_rand ...
                                                                                                                                 , r_rand ...
                                                                                                                                 , r_bb ...
                                                                                                                                 , show )
    %%%% OUTPUT: 
    %%%% TrajPointsBin is similar to OCTREE.PointsBin, i.e., it is a vecotr
    %%%%    long as the list of trajectory points, and each element is the
    %%%%    index of the corresponding Bin in OG. 
    %%%% TrajPoints is similar to OCTREE.Points, i.e., it is a vector long
    %%%%    as the list of trajectory points, and each element is a valid point
    %%%%    inside at least to a Bin. They preserve the trajectory point order!
    %%%% TrajBinOrder is a vector with the list of the used Bin, ordered as
    %%%%    the trajectory
    %%%% TrajBinNElements is a vector long as TrajBinOrder, the number of
    %%%%    trajectory point for each used bin, ordered as the the trajectory
    %%%% HocBinMean,HocBinSum, HocBinMax, HocBinMin: number of occurences
    %%%%    for each bin
        

%1) Create the common octree ( human grid + trajecotry )
    %1.1) add some random points around the trajectory
    rtraj_pts = add_random_point( traj_pts, n_rand, r_rand );
    
    %1.2) Create the random points around the trajectory
    OCTREE = OcTree( [ og_pts; rtraj_pts  ], 'binCapacity', round( length([ og_pts; rtraj_pts  ]) / decimation_factor ) ); 
    OCTREE.shrink

    %1.3) Select the bins collecting both human and robot points 
    index_ogpts    = 1:length( og_pts );
    index_rtajpts  = length( og_pts ) + ( 1:length(rtraj_pts ) );
    CommonBins     = intersect( OCTREE.PointBins( index_ogpts )   ...  %i.e., indexes of the bin of the human points  ...
                              , OCTREE.PointBins( index_rtajpts ) ...  %i.e., indexes of the bin of the trajectory (with random points) 
                              );                                       %i.e., extract the bins that contains at both TRAj and HUMAN points
    
%2) Create a Octree with only the human points that are stored in the
% CommonBins
    
    %2.1) look for the points of the human that are in the common bins
    OgPoints       = OCTREE.Points( index_ogpts, : );
    index_ogpts_   = [];
    for iBin = 1:length( CommonBins )
        index_ogpts_   = [ index_ogpts_   ; find( OCTREE.PointBins( index_ogpts ) == CommonBins( iBin ) ); ];
    end
    OgCommonPoints   = OgPoints( index_ogpts_, : );                     % i.e., just the points that belong to the same Bin of the Trajectory
    OgHoc            = h_oc( index_ogpts_ );                           % i.e., just the occupancy of the points that belongs to the same Bin of the Trajectory
    
    %2.2) Create the new OcTree given the OcTree
    OG = OcTree( OgCommonPoints, 'maxSize', 1.5 * r_bb, 'minSize', r_bb ); 
    
    %2.3) Identify the indexes of the Bins that sotres the points
    OgPointBins = unique( OG.PointBins ); %unique return the vector ascendent sorted 
    
    %%%% TrajPointsBin is similar to OCTREE.PointsBin, i.e., it is a vecotr
    %%%% long as the list of trajectory points, and each element is the
    %%%% index of the corresponding Bin in OG. 
    TrajPointsBin = zeros( length(traj_pts), 1 ); 
    for iPt = 1:length(traj_pts)
        traj_pt = traj_pts( iPt, : );
        for iBin=length( OgPointBins ):-1:1
            if ( traj_pt(1) >= OG.BinBoundaries(OgPointBins( iBin ),1) ) && ( traj_pt(1) <= OG.BinBoundaries(OgPointBins( iBin ),4) ) ...
            && ( traj_pt(2) >= OG.BinBoundaries(OgPointBins( iBin ),2) ) && ( traj_pt(2) <= OG.BinBoundaries(OgPointBins( iBin ),5) ) ...
            && ( traj_pt(3) >= OG.BinBoundaries(OgPointBins( iBin ),3) ) && ( traj_pt(3) <= OG.BinBoundaries(OgPointBins( iBin ),6) )
                TrajPointsBin( iPt ) = OgPointBins( iBin );
                break;
            end
        end
    end
    idxValidPoints = find( TrajPointsBin > 0 );
    TrajPointsBin  = TrajPointsBin( idxValidPoints );
    
    %%% TrajPoints is similar to OCTREE.Points, i.e., it is a vector long
    %%% as the list of trajectory points, and each element is a valid point
    %%% inside at least to a Bin. They preserve the trajectory point order!
    TrajPoints = traj_pts( idxValidPoints );
    
    %%%% TrajBinOrder is a vector with the list of the used Bin, ordered as
    %%%%    the trajectory
    [ ~, ia, ~ ]  = unique( TrajPointsBin );
    TrajBinOrder  = TrajPointsBin( sort( ia ) );
    
    %%%% TrajBinNElements is a vector long as TrajBinOrder, it stores the number of
    %%%% trajectory point for each used bin, ordered as the the trajectory
    TrajBinNElements = zeros( size( TrajBinOrder ) );
    for iBin = 1:length( TrajBinOrder )
        TrajBinNElements( iBin ) = length( find( TrajPointsBin == TrajBinOrder( iBin ) ) );
    end
    
    %%%% HocBinMean is a vector long as TrajBinOrder, it stores the number of
    %%%% the mean human occupancy  occurrences for each used bin, ordered as the the trajectory
    HocBinMean = zeros( size( TrajBinOrder ) );
    HocBinSum  = zeros( size( TrajBinOrder ) );
    HocBinMax  = zeros( size( TrajBinOrder ) );
    HocBinMin  = zeros( size( TrajBinOrder ) );
    for iBin = 1:length( TrajBinOrder )
        OgHoc_ = OgHoc( find( OG.PointBins == TrajBinOrder( iBin ) ) );
        if isempty( OgHoc_ )
            TrajPointsBin
        end
        HocBinMean( iBin ) = mean( OgHoc_ );
        HocBinSum ( iBin ) = sum ( OgHoc_ );
        HocBinMax ( iBin ) = max ( OgHoc_ );
        HocBinMin ( iBin ) = min ( OgHoc_ );
    end
    
    
    if show == 1
        figure
        subplot(2,1,1)
        boxH = OCTREE.plot;
        cols = lines(OCTREE.BinCount);
        doplot3 = @(p,varargin)plot3(p(:,1),p(:,2),p(:,3),varargin{:});
        for i = 1:OCTREE.BinCount
         set(boxH(i),'Color',cols(i,:),'LineWidth', 1+OCTREE.BinDepths(i))
         pts = [ og_pts; rtraj_pts ];
         doplot3(pts(OCTREE.PointBins==i,:),'.','Color',cols(i,:))
        end
        axis image, view(3)
        
        subplot(2,1,2)
        if OG.BinCount > 1 
            boxHN = OG.plot;
            cols = lines(OG.BinCount);
            doplot3 = @(p,varargin)plot3(p(:,1),p(:,2),p(:,3),varargin{:});
            for i = 1:OG.BinCount
             set(boxHN(i),'Color',cols(i,:),'LineWidth', 1+OG.BinDepths(i))
             doplot3(OgCommonPoints(OG.PointBins==i,:),'.','Color',cols(i,:))
            end
            axis image, view(3)
        else
            disp('Attention! No collision between human and trajectory');
        end
    end
end