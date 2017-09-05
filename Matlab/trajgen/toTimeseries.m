function [ tss, TExperimentStart, TExperimentEnd, THumanSingleTask ] = toTimeseries( opennitracker_data, HumanTaskRepetitions )

persistent frames_used tdd ;
if isempty( frames_used )
    frames_used ={ '/skel/SpineBase' ...
                 , '/skel/SpineMid'  ...
                 , '/skel/Neck' ...
                 , '/skel/Head' ...
                 , '/skel/ShoulderLeft' ...
                 , '/skel/ElbowLeft' ...
                 , '/skel/WristLeft' ...
                 , '/skel/HandLeft' ...
                 , '/skel/ShoulderRight' ...
                 , '/skel/ElbowRight' ...
                 , '/skel/WristRight' ...
                 , '/skel/HandRight' ...
                 , '/skel/HipLeft' ...
                 , '/skel/KneeLeft' ...
                 , '/skel/AnkleLeft' ...
                 , '/skel/FootLeft' ...
                 , '/skel/HipRight' ...
                 , '/skel/KneeRight' ...
                 , '/skel/AnkleRight' ...
                 , '/skel/FootRight' ...
                 , '/skel/SpineShoulder' ...
                 , '/skel/HandTipLeft' ...
                 , '/skel/ThumbLeft' ...
                 , '/skel/HandTipRight' ...
                 , '/skel/ThumbRight' };

tdd = cell( length( frames_used ), 2 );

end

max_allowed_samples = length(opennitracker_data);
tss = cell( length( frames_used ), 3 );
for i=1:length( frames_used )
    tdd{i,1}=zeros(max_allowed_samples,1);
    tdd{i,2}=zeros(max_allowed_samples,3);
end
tddIndex = zeros(length( frames_used ),1);

TExperimentStart = opennitracker_data{1}{1};
TExperimentEnd   = opennitracker_data{end}{1};
THumanSingleTask = (TExperimentEnd - TExperimentStart) / HumanTaskRepetitions;
t0 = 0;
for t = 1:length( opennitracker_data )
    
    time_           = opennitracker_data{t}{1} - TExperimentStart; 
    if time_ < THumanSingleTask 
        t0 = t;
    end
    frames_names_   = opennitracker_data{t0}{2};
    frames_data_    = opennitracker_data{t0}{3};
    
    for iFrameUsed = 1:length( frames_used )
        for iFrameName =1:length( frames_names_ )
            if strcmp( frames_used{iFrameUsed} , frames_names_{iFrameName} ) ...
            && sqrt( frames_data_( iFrameName, 1 ).^2 + frames_data_( iFrameName, 2 ).^2 + frames_data_( iFrameName, 3 ).^2 ) > 0.005 
               tddIndex(iFrameUsed) = tddIndex(iFrameUsed) + 1;
               tdd{iFrameUsed,1}(tddIndex(iFrameUsed))   = time_; 
               tdd{iFrameUsed,2}(tddIndex(iFrameUsed),:) = frames_data_( iFrameName, : );
               
               break;
            end
        end
    end
end

idxFramesUsed = find( tddIndex > 0 );
TExperimentEnd  = TExperimentEnd-TExperimentStart;
TExperimentStart  = 0;
for iTss = 1:length( frames_used )
    if iTss < length( idxFramesUsed )
       idxFrameUsed = idxFramesUsed(iTss);
    else
        idxFrameUsed = idxFramesUsed(end);
    end
    
    [t,IA,~] = unique( tdd{idxFrameUsed,1}(1:tddIndex(idxFrameUsed)) );
    
    tss{iTss,1} = spline  ( t, tdd{idxFrameUsed,2}(IA,1) );
    tss{iTss,2} = spline  ( t, tdd{idxFrameUsed,2}(IA,2) );
    tss{iTss,3} = spline  ( t, tdd{idxFrameUsed,2}(IA,3) );
end

