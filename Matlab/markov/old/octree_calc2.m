function [OG, TRAJ] = octree_calc( pts, index_ogpts, index_trajpts, decimation_factor, h_oc, show )

%1) Create the cotree
    %OCTREE = OcTree(pts, 'binCapacity', round( length(pts) / decimation_factor ) ); 
    OCTREE = OcTree(pts, 'minSize', 0.2, 'maxSize', 0.25 ); 
    OCTREE.shrink

%2) Structures declaration 
    TRAJ = struct( 'BinCount'       , 0  ... % as in OCTREE, i.e., the number of bin describing the object
                 , 'BinBoundaries'  , [] ... % as in OCTREE, i.e., the position of the bins ( BinCount x 3 )
                 , 'BinDepths'      , [] ... % as in OCTREE, i.e., the depth of each bin ( BinCount x 1 )
                 , 'BinParents'     , [] ... % as in OCTREE, i.e., the parent index of each bin ( BinCount x 1 )
                 , 'Points'         , [] ... % as in OCTREE, i.e., the points of the object 
                 , 'PointBins'      , [] ... % as in OCTREE, i.e., the bin index for each point 
                 , 'PointBinsUnique', [] ... % transient variable, i.e., BinCount = length( PointBinsUnique ). PointBinsUnique is a vector ( BinCount x 1 ) with the index of the bins used.
                 , 'BinNSamples'    , [] ... % number of object points in each bin ( BinCount x 1 )
                 );

    OG   = struct( 'BinCount'       , 0  ... % as in OCTREE, i.e., the number of bin describing the object
                 , 'BinBoundaries'  , [] ... % as in OCTREE, i.e., the position of the bins ( BinCount x 3 )
                 , 'BinDepths'      , [] ... % as in OCTREE, i.e., the depth of each bin ( BinCount x 1 )
                 , 'BinParents'     , [] ... % as in OCTREE, i.e., the parent index of each bin ( BinCount x 1 )
                 , 'Points'         , [] ... % as in OCTREE, i.e., the points of the object 
                 , 'PointBins'      , [] ... % as in OCTREE, i.e., the bin index for each point 
                 , 'PointBinsUnique', [] ... % transient variable, i.e., BinCount = length( PointBinsUnique ). PointBinsUnique is a vector ( BinCount x 1 ) with the index of the bins used.
                 , 'BinNSamples'    , [] ... % number of object points in each bin ( BinCount x 1 )
                 , 'BinSum'         , [] ... % sum of the occurencies given by each object points in each bin ( BinCount x 1 ) 
                 , 'BinMean'        , [] ... % mean of the occurencies given by each object points in each bin ( BinCount x 1 ) 
                 , 'BinMax'         , [] ... % max of the occurencies given by each object points in each bin ( BinCount x 1 ) 
                 , 'BinMin'         , [] ... % min of the occurencies given by each object points in each bin ( BinCount x 1 ) 
                 );

% 3) TRAJ definition
    OgPointBins                 = OCTREE.PointBins( index_ogpts );      %i.e., indexes of the bin of the human points 
    TrajPointBins               = OCTREE.PointBins( index_trajpts );    %i.e., indexes of the bin of the trajecotry points 
    
    % [C,IA,IC] = unique(A)        ---> C = A(IA,:) and A = C(IC,:)
    [ OgPointBinsUnique,iA,iC ] = unique( OgPointBins );                %i.e., purge duplicated indexes from OgPointBins (attention, unique return a sorted vector)
    OgPointBinsUnique           = OgPointBins( sort( iA ) );            %i.e., reorder w.r.t to the bin indexes
    
    % [E,ID,IE] = unique(D)        ---> E = D(ID,:) and D = E(IE,:) 
    [ TrajPointBinsUnique, iD, iE ] = unique( TrajPointBins );          %i.e., purge duplicated indexes from TrajPointBins (attention, unique return a sorted vector)
    TrajPointBinsUnique             = TrajPointBins( sort( iD ) );      %i.e., reorder w.r.t to the bin indexes
    
    % [F,IC2,IE2] = intersect(C,E) ---> F = C(IC2) and F = E(IE2)
    [ PointBins, iC2, iE2 ] = intersect( OgPointBinsUnique, TrajPointBinsUnique );     %i.e., extract the bins that contains at both TRAj and HUMAN points
    PointBins               = TrajPointBinsUnique( sort( iE2 ) );

    TRAJ.PointBinsUnique = PointBins; 
    TRAJ.BinCount        = length( TRAJ.PointBinsUnique );
    TRAJ.BinBoundaries   = OCTREE.BinBoundaries( TRAJ.PointBinsUnique, : );
    TRAJ.BinDepths       = OCTREE.BinDepths( TRAJ.PointBinsUnique );
    TRAJ.BinParents      = OCTREE.BinParents( TRAJ.PointBinsUnique );
    TRAJ.BinNSamples     = zeros( TRAJ.BinCount, 1 );

    %%% tmp var
    Points_ = OCTREE.Points( index_trajpts, : );
    PointBins_ = OCTREE.PointBins( index_trajpts, : );
    PointIndexes_ = [];
    for k = 1:TRAJ.BinCount
        ipts = find( PointBins_ == TRAJ.PointBinsUnique( k ) );
        PointIndexes_ = [ PointIndexes_; ipts ];
        TRAJ.BinNSamples( k ) = length( ipts );
    end
    TRAJ.Points = Points_( PointIndexes_, : );
    TRAJ.PointBins = PointBins_( PointIndexes_, : );
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % 4) OG definition
    OG.PointBinsUnique  = TRAJ.PointBinsUnique;
    OG.BinCount         = TRAJ.BinCount;
    OG.BinBoundaries    = TRAJ.BinBoundaries;
    OG.BinDepths        = TRAJ.BinDepths;
    OG.BinParents       = TRAJ.BinParents;

    OG.Points      = [];
    OG.PointBins   = [];
    OG.BinNSamples = zeros( TRAJ.BinCount, 1 );
    OG.BinSum      = zeros( TRAJ.BinCount, 1 );
    OG.BinMean     = zeros( TRAJ.BinCount, 1 );
    OG.BinMax      = zeros( TRAJ.BinCount, 1 );
    OG.BinMin      = zeros( TRAJ.BinCount, 1 );
    
    Points_ = OCTREE.Points( index_ogpts, : );
    PointBins_ = OCTREE.PointBins( index_ogpts, : );
    PointIndexes_ = [];
    for k = 1:OG.BinCount
        ipts = find( PointBins_ == OG.PointBinsUnique( k ) );
        PointIndexes_ = [ PointIndexes_; ipts ];
        if isempty( ipts )
            OG.BinSum ( k ) = 0; %1
            OG.BinMean( k ) = 0; %2
            OG.BinMax ( k ) = 0; %3
            OG.BinMin ( k ) = 0; %4
            OG.BinNsamples( k ) = 0; %5
        else
            OG.BinSum     ( k ) = sum ( h_oc( ipts ) ); %1
            OG.BinMean    ( k ) = mean( h_oc( ipts ) ); %2
            OG.BinMax     ( k ) = max ( h_oc( ipts ) ); %3
            OG.BinMin     ( k ) = min ( h_oc( ipts ) ); %4
            OG.BinNsamples( k ) = length ( ipts );      %5
        end
    end
    OG.Points    = Points_( PointIndexes_, : );
    OG.PointBins = PointBins_( PointIndexes_, : );


    
    if show == 1
        figure
        subplot(2,1,1)
        boxH = OCTREE.plot;
        cols = lines(OCTREE.BinCount);
        doplot3 = @(p,varargin)plot3(p(:,1),p(:,2),p(:,3),varargin{:});
        for i = 1:OCTREE.BinCount
         set(boxH(i),'Color',cols(i,:),'LineWidth', 1+OCTREE.BinDepths(i))
         doplot3(pts(OCTREE.PointBins==i,:),'.','Color',cols(i,:))
        end
        axis image, view(3)
        
        subplot(2,1,2)
        
        OCTREENEW = OcTree([OG.Points; TRAJ.Points], 'minSize', 0.2, 'maxSize', 0.25 ); 
        OCTREENEW.shrink

        boxHN = OCTREENEW.plot;
        cols = lines(OCTREENEW.BinCount);
        doplot3 = @(p,varargin)plot3(p(:,1),p(:,2),p(:,3),varargin{:});
        for i = 1:OCTREENEW.BinCount
         set(boxHN(i),'Color',cols(i,:),'LineWidth', 1+OCTREENEW.BinDepths(i))
         doplot3(pts(OCTREENEW.PointBins==i,:),'.','Color',cols(i,:))
        end
        axis image, view(3)
    end
    
end