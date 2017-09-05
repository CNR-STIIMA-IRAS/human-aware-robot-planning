clear all
close all
clc

%%
% idx, x, y, z, occurrency
% oc: numero di occorrenze entando nella cella una sola volta
n_frame= 619;
ds     = 0.01;
points = load('points.mat', '-ascii');
id=points(:,1);
h_x=points(:,2);
h_y=points(:,3);
h_z=points(:,4);
h_oc=points(:,5);
n_points = length(h_x);

%%
Pmax = [];
BinCount = [];
Texpected = [];
Tworst = [];
for oo=1:100
    
    clearvars -except h_x h_y h_z h_oc n_points id ds n_frame Texpected Tworst Pmax BinCount
    ext = -0.5 + rand();
    %trajecory definition
    traj_x= linspace(min(h_x)-ext, max(h_x)-0.5*ext, 100)';
    traj_y= linspace(min(h_y)-ext, max(h_y)+ext, 100)';
    traj_z= linspace(min(h_z)-ext, max(h_z)+ext, 100)';


    %%
    pts=[h_x' traj_x'; h_y' traj_y'; h_z' traj_z']';
    OCTREE = OcTree(pts, 'binCapacity', round( length(pts) / 100 ) ); 
    OCTREE.shrink
%     figure
%     boxH = OCTREE.plot;
%     cols = lines(OCTREE.BinCount);
%     doplot3 = @(p,varargin)plot3(p(:,1),p(:,2),p(:,3),varargin{:});
%     for i = 1:OCTREE.BinCount
%      set(boxH(i),'Color',cols(i,:),'LineWidth', 1+OCTREE.BinDepths(i))
%      doplot3(pts(OCTREE.PointBins==i,:),'.','Color',cols(i,:))
%     end
%     axis image, view(3)

    %%

    % 1) Index corresponding to the OG and the TRAJ
    index_ogpts     = 1:length(h_x);
    index_trajpts   = length(h_x) + 1: length(pts);

    % 2) Structures declaration 
    TRAJ = struct( 'BinCount'       , 0  ... % as in OCTREE, i.e., the number of bin describing the object
                 , 'BinBoundaries'  , [] ... % as in OCTREE, i.e., the position of the bins ( BinCount x 3 )
                 , 'BinDepths'      , [] ... % as in OCTREE, i.e., the depth of each bin ( BinCount x 1 )
                 , 'BinParents'     , [] ... % as in OCTREE, i.e., the parent index of each bin ( BinCount x 1 )
                 , 'Points'         , [] ... % as in OCTREE, i.e., the points of the object 
                 , 'PointBins'      , [] ... % as in OCTREE, i.e., the bin index for each point 
                 , 'PointBinsUnique', [] ... % transient variable, i.e., BinCount = length( PointBinsUnique ). PointBinsUnique is a vector ( BinCount x 1 ) with the index of the bins used.
                 , 'BinNSamples'    , [] ... % number of object points in each bin ( BinCount x 1 )
                 , 'PointIndexes'   , [] ... % 
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
    TrajPointBins               = OCTREE.PointBins( index_trajpts );
    [ PointBinsUnique, iA, iC ] = unique( TrajPointBins );

    TRAJ.PointBinsUnique        = TrajPointBins( sort( iA ) );
    TRAJ.BinCount               = length( TRAJ.PointBinsUnique );
    TRAJ.BinBoundaries          = OCTREE.BinBoundaries( TRAJ.PointBinsUnique, : );
    TRAJ.BinDepths              = OCTREE.BinDepths( TRAJ.PointBinsUnique );
    TRAJ.BinParents             = OCTREE.BinParents( TRAJ.PointBinsUnique );
    TRAJ.Points                 = OCTREE.Points( index_trajpts, : );
    TRAJ.PointBins              = OCTREE.PointBins( index_trajpts, : );
    TRAJ.BinNSamples            = zeros( TRAJ.BinCount, 1 );
    TRAJ.PointIndexes           = [];
    for k = 1:TRAJ.BinCount
        ipts = find( TRAJ.PointBins == TRAJ.PointBinsUnique( k ) );
        TRAJ.PointIndexes = [ TRAJ.PointIndexes; ipts ];
        TRAJ.BinNSamples( k ) = length( ipts );
    end
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
    OG.PointIndexes= [];
    disp(['OG.BinCount:',num2str( OG.BinCount ) ])
    disp(['TRAJ.BinCount:',num2str( TRAJ.BinCount )])
    for k = 1:OG.BinCount
        ipts = find( OCTREE.PointBins( index_ogpts ) == OG.PointBinsUnique( k ) );
        OG.PointIndexes = [ OG.PointIndexes; ipts ];
        if isempty( ipts )
            OG.BinSum ( k ) = 0; %1
            OG.BinMean( k ) = 0; %2
            OG.BinMax ( k ) = 0; %3
            OG.BinMin ( k ) = 0; %4
            OG.BinNsamples( k ) = 0; %5
        else
            OG.Points    = [ OG.Points; OCTREE.Points( ipts, : ) ];
            OG.PointBins = [ OG.PointBins; OCTREE.PointBins( ipts, : ) ];

            OG.BinSum     ( k ) = sum ( h_oc( ipts ) ); %1
            OG.BinMean    ( k ) = mean( h_oc( ipts ) ); %2
            OG.BinMax     ( k ) = max ( h_oc( ipts ) ); %3
            OG.BinMin     ( k ) = min ( h_oc( ipts ) ); %4
            OG.BinNsamples( k ) = length ( ipts );      %5
        end
    end



    %%
    % Markov

    P = zeros( OG.BinCount, OG.BinCount );
    for k=1:TRAJ.BinCount-1
        %disp([ 'Bin Index:', num2str(OG.PointBinsUnique( k ) ), ' Mean Occupancy: ', num2str( OG.BinMean( k ) / n_frame ) ]);
        P( k, k   ) = OG.BinMean( k ) / n_frame;
        P( k, k+1 ) = 1.0 - OG.BinMean( k ) / n_frame;
    end
    P( OG.BinCount, OG.BinCount ) = 1;

    Q = P(1:OG.BinCount-1, 1:OG.BinCount-1);
    N = ( eye( OG.BinCount-1 ) - Q)^-1;
    R = P( 1:OG.BinCount-1, OG.BinCount );
    B = N*R;

    Thuman      = 12.0;
    Tnom        = 6.0;
    Tstop_robot = 1.0;
    delay       = zeros(TRAJ.BinCount, 1);
    for k=1:TRAJ.BinCount-1
        Tstop_human = P(1,k) * Thuman;
        if Tstop_human > Tstop_robot
            Tstop = Tstop_human;
        else
            Tstop = Tstop_robot;
        end
        delay( k ) = Tstop;
    end
    delay( end ) = TRAJ.BinNSamples(end)/sum(TRAJ.BinNSamples(end)) * OG.BinMean( end ) / n_frame * Thuman;
    if Tstop > max( h_oc( OG.PointIndexes ) ) /n_frame * Thuman
        Tstop_max = Tstop;
    else
        Tstop_max = max( h_oc( OG.PointIndexes ) ) / n_frame * Thuman;
    end
    Pmax      = [ Pmax;     max( max( diag( Q )) ) ];
    BinCount  = [ BinCount; TRAJ.BinCount ];
    Texpected = [ Texpected ; Tnom + ( N(1,:) - 1 ) * delay(1:end-1) + delay(end); ];
    Tworst    = [ Tworst;     Tnom + Tstop_max + Tstop ];

    disp(['Texpected :',num2str(Texpected(end))])
    disp(['Tworst    :',num2str(Tworst(end))])
    
end
% % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % stop
% % % % % % % % % % % % % % % % % % % % % % % %% 
% % % % % % % % % % % % % % % % % % % % % % % figure
% % % % % % % % % % % % % % % % % % % % % % % subplot(2,1,1)
% % % % % % % % % % % % % % % % % % % % % % % plot( OGBinSum );
% % % % % % % % % % % % % % % % % % % % % % % title('Occorrence Sum')
% % % % % % % % % % % % % % % % % % % % % % % subplot(2,1,2)
% % % % % % % % % % % % % % % % % % % % % % % plot ( OGBinMean );  hold on;
% % % % % % % % % % % % % % % % % % % % % % % plot ( OGBinMax ); hold on;
% % % % % % % % % % % % % % % % % % % % % % %  legend('mean','max')
% % % % % % % % % % % % % % % % % % % % % % % hold on;
% % % % % % % % % % % % % % % % % % % % % % % stop
% % % % % % % % % % % % % % % % % % % % % % % %%
% % % % % % % % % % % % % % % % % % % % % % % %plot traj
% % % % % % % % % % % % % % % % % % % % % % % figure
% % % % % % % % % % % % % % % % % % % % % % % plot3(traj_x,traj_y,traj_z)
% % % % % % % % % % % % % % % % % % % % % % % hold on
% % % % % % % % % % % % % % % % % % % % % % % grid on
% % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % %figure
% % % % % % % % % % % % % % % % % % % % % % % scatter3(h_x,h_y,h_z,ones(size(h_x,1),1),h_oc/max(h_oc))
% % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % figure()
% % % % % % % % % % % % % % % % % % % % % % % % n=30000;
% % % % % % % % % % % % % % % % % % % % % % % % scatter3(x(1:n),y(1:n),z(1:n),ones(n,1),oc(1:n)/max(oc));
% % % % % % % % % % % % % % % % % % % % % % % % grid on
% % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % %%
% % % % % % % % % % % % % % % % % % % % % % % %time
% % % % % % % % % % % % % % % % % % % % % % % traj_time=7.2;
% % % % % % % % % % % % % % % % % % % % % % % h_time=3;
% % % % % % % % % % % % % % % % % % % % % % % r_start=0.5;
% % % % % % % % % % % % % % % % % % % % % % % r_stop=0.5;
% % % % % % % % % % % % % % % % % % % % % % % C=traj_time/h_time;
% % % % % % % % % % % % % % % % % % % % % % % Tmax=traj_time+h_time+r_start * ceil(C);

%
% % % % % % % % % % % % % % % % % % % % % % % traj_prob=traj_time/length(traj_x)*ones(length(traj_x),1);





