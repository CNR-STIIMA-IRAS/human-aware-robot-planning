function [time, positions, ts ] = extractData( msgs, startTime, frameid )

time = zeros( length( msgs ), 1 );
positions = zeros( length( msgs ), 3 );
for iMsg = 1:length(msgs)
    time(iMsg) = msgs(iMsg).Transforms.Header.Stamp.Sec + msgs(iMsg).Transforms.Header.Stamp.Nsec / 1.e9;
    positions(iMsg,:) = [ msgs(iMsg).Transforms.Transform.Translation.X ...
                        , msgs(iMsg).Transforms.Transform.Translation.Y ...
                        , msgs(iMsg).Transforms.Transform.Translation.Z ];
end

[time, idx] = sort(time);
positions = positions( idx, : );

time = time - startTime;

ts = timeseries( positions, time );
ts.Name = frameid;