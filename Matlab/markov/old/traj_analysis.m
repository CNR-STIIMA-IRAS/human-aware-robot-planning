clear all
close all
clc

%%
% idx, x, y, z, occurrency
% oc: numero di occorrenze entando nella cella una sola volta
n_frame=619;
ds = 0.01;
points=load('points.mat', '-ascii');
id=points(:,1);
h_x=points(:,2);
h_y=points(:,3);
h_z=points(:,4);
h_oc=points(:,5);
n_points = length(h_x);

%%%%
index_oc_1                  = find( h_oc == 1 );
index_oc_greater_than_1     = find( h_oc > 1 );
n_points_oc_1               = length( index_oc_1 );
n_points_oc_greater_than_1  = length( index_oc_greater_than_1 );

%%%% 
cum_points_to_add           = cumsum( h_oc( index_oc_greater_than_1 ) );
n_points_to_add             = cum_points_to_add (end);
points_to_add               = random( 'Normal', 0, 0.9 * ds/3, [ n_points_to_add, 3 ] );

%%%%
n_total_points              = n_points + n_points_to_add;
hh_x                        = zeros( n_total_points, 1 );
hh_y                        = zeros( n_total_points, 1 );
hh_z                        = zeros( n_total_points, 1 );


%% 
% add points at the end of the list
hh_x(1:n_points_oc_1 ) = h_x( index_oc_1 );
hh_y(1:n_points_oc_1 ) = h_y( index_oc_1 );
hh_z(1:n_points_oc_1 ) = h_z( index_oc_1 );

s=1;
for i=1:n_points_oc_greater_than_1
    e      = cum_points_to_add(i);
    range  = s : e;
    s      = e+1;
    
    hh_x( n_points_oc_1 + range ) = h_x( index_oc_greater_than_1(i) ) + points_to_add( range, 1 );
    hh_y( n_points_oc_1 + range ) = h_y( index_oc_greater_than_1(i) ) + points_to_add( range, 2 );
    hh_z( n_points_oc_1 + range ) = h_z( index_oc_greater_than_1(i) ) + points_to_add( range, 3 );
end


%%
%trajecory definition
ext=0.5;
traj_x=[min(h_x)-ext:0.01:max(h_x)+ext];
traj_y=[min(h_y)-ext:0.01:max(h_y)+ext];
traj_z=[min(h_z)-ext:0.01:max(h_z)+ext];
min_value=min([length(traj_x),length(traj_y),length(traj_z)]);
traj_x=traj_x(1:min_value)';
traj_y=traj_y(1:min_value)';
traj_z=traj_z(1:min_value)';

%%
pts=[hh_x' traj_x';hh_y' traj_y';hh_z' traj_z']';
%OT = OcTree(pts,'binCapacity',20)%,'minSize',1)  
%binCapacity = round( n_total_points / 25 )
OT = OcTree(pts,'binCapacity', round( n_total_points / 250 ), 'minSize',0.05)  
%OT = OcTree( pts, 'maxSize',0.25)  
OT.shrink
figure
boxH = OT.plot;
cols = lines(OT.BinCount);
doplot3 = @(p,varargin)plot3(p(:,1),p(:,2),p(:,3),varargin{:});
for i = 1:OT.BinCount
 set(boxH(i),'Color',cols(i,:),'LineWidth', 1+OT.BinDepths(i))
 doplot3(pts(OT.PointBins==i,:),'.','Color',cols(i,:))
end
axis image, view(3)

%%
clear OTPointBins OTPoints OTSum index_trj OTPoints OTPointBins OTBinBoundaries  OTBinDepths OTBinParents OTPointBinsUnique OTBinCount 
index_trj       = length(hh_x) + 1: length(pts);
OTPoints        = OT.Points( index_trj, : );
OTPointBins     = OT.PointBins( index_trj );
[ OTPointBinsUnique, ia, ic ] = unique( OTPointBins );
OTBinCount      = length( OTPointBinsUnique );
OTBinBoundaries = OT.BinBoundaries( OTPointBinsUnique, : );
OTBinDepths     = OT.BinDepths( OTPointBinsUnique );
OTBinParents    = OT.BinParents( OTPointBinsUnique );
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

OTBinSum        = zeros( OTBinCount, 1 );
for k = 1:OTBinCount
    ips = find( OTPointBins == OTPointBinsUnique( k ) );
    ps  = OTPoints( ips, : );
    OTBinSum ( k ) = size( ps, 1 );
end

OTBinParentsChain = cell( OTBinCount, 1 );
for k = 1:OTBinCount
    OTBinParentsChain{k,1} = [ OTPointBinsUnique( k ) ];
    while 1
       if OTBinParentsChain{k,1}(end) > 0 ...
       && OTBinParentsChain{k,1}(end) < length( OT.BinParents )
        OTBinParentsChain{k,1} = [ OTBinParentsChain{k,1}, OT.BinParents( OTBinParentsChain{k,1}(end) ) ];
       else
           break;
       end
    end
    OTBinParentsChain{k,1}
end

figure
plot( OTBinSum );
%%
%plot traj
figure
plot3(traj_x,traj_y,traj_z)
hold on
grid on

%figure
scatter3(h_x,h_y,h_z,ones(size(h_x,1),1),h_oc/max(h_oc))

% figure()
% n=30000;
% scatter3(x(1:n),y(1:n),z(1:n),ones(n,1),oc(1:n)/max(oc));
% grid on

%%
%time
traj_time=7.2;
h_time=3;
r_start=0.5;
r_stop=0.5;
C=traj_time/h_time;
Tmax=traj_time+h_time+r_start * ceil(C);

%
traj_prob=traj_time/length(traj_x)*ones(length(traj_x),1);





