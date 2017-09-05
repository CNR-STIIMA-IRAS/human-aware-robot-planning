clear all
close all
clc

%%
% idx, x, y, z, occurrency
% oc: numero di occorrenze entando nella cella una sola volta
n_frame= 619;
ds     = 0.01;
points = load('C:\\Users\\NicolaPedrocchi\\OneDrive - C.N.R. ITIA\\Progetti\\shared-vb\\Sperim_Letters\\points.mat', '-ascii');
%points = load('points.mat', '-ascii');
id=points(:,1);
h_x=points(:,2);
h_y=points(:,3);
h_z=points(:,4);
h_oc=points(:,5);
n_points = length(h_x);

%%
%trajecory definition
ext=0.2;
traj_x=[min(h_x)-ext:0.01:max(h_x)+ext];
traj_y=[min(h_y)-ext:0.01:max(h_y)+ext];
traj_z=[min(h_z):0.01:max(h_z)+ext];
min_value=min([length(traj_x),length(traj_y),length(traj_z)]);
traj_x=traj_x(1:min_value)';
traj_y=traj_y(1:min_value)';
traj_z=traj_z(1:min_value)';

%%
pts=[h_x' traj_x'; h_y' traj_y'; h_z' traj_z']';
OT = OcTree(pts,'minSize',0.015); 
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
index_pts       = 1:length(h_x);
index_trj       = length(h_x) + 1: length(pts);
[ OTPointBinsUnique, ia, ic ] = unique( OT.PointBins( index_trj ) );
OTBinCount      = length( OTPointBinsUnique );
OTBinBoundaries = OT.BinBoundaries( OTPointBinsUnique, : );
OTBinDepths     = OT.BinDepths( OTPointBinsUnique );
OTBinParents    = OT.BinParents( OTPointBinsUnique );
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

OTPoints    = [];
OTPointBins = [];
OTBinSum    = zeros( OTBinCount, 1 );
OTBinMean   = zeros( OTBinCount, 1 );
OTBinMax    = zeros( OTBinCount, 1 );
OTBinMin    = zeros( OTBinCount, 1 );
for k = OTBinCount
    ipts = find( OT.PointBins( index_pts ) == OTPointBinsUnique( k ) );
    if isempty( ipts )
        OTBinSum ( k ) = 0;
        OTBinMean( k ) = 0;
        OTBinMax ( k ) = 0;
        OTBinMin ( k ) = 0;
        OTBinNsamples( k ) = 0;
    else
        OTPoints = [ OTPoints; OT.Points( ipts, : ) ];
        OTPointBins = [ OTPointBins; OT.PointBins( ipts, : ) ];
        OTBinSum ( k ) = sum ( h_oc( ipts ) );
        OTBinNsamples( k ) = length ( ipts );
        OTBinMean( k ) = mean( h_oc( ipts ) );
        OTBinMax ( k ) = max ( h_oc( ipts ) );
        %OTBinMin ( k ) = min ( h_oc( ipts ) );
    end
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
subplot(2,1,1)
plot( OTBinSum );
title('Occorrence Sum')
subplot(2,1,2)
plot ( OTBinMean );  hold on;
plot ( OTBinMax ); hold on;
 legend('mean','max')
hold on;
stop
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





