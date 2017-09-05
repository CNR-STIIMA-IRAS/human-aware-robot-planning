function ret = MC( BinCount, BinMean, BinNSamples, combinations, Thuman, Tstop_robot, Tnom, n_frame )

    persistent lDiag;
    if isempty(lDiag) 
        lDiag = logical( eye ( BinCount  , BinCount ) );
    end
    
    persistent uDiag;
    if isempty(uDiag) 
        uDiag = logical( triu( ones(BinCount,BinCount ), 1 ) - triu( ones(BinCount,BinCount), 2 ) );
    end
    
    persistent comb;
    if isempty(comb) 
        comb = zeros( BinCount, 1 );
    end
            



    data_mean =  struct ( 'P'  , eye( BinCount  , BinCount )  ... % Probability matrix
                        , 'Q'  , zeros( BinCount-1, BinCount-1 )  ... % absortion matrix
                        , 'N'  , zeros( BinCount-1, BinCount-1 )  ... % fundamental matrix
                        , 'R'  , zeros( BinCount-1, 1 ) ...
                        , 'B'  , zeros( BinCount-1, 1 ) ...
                        );
%     if BinCount < 3
%         for k=1:BinCount
%             if any( combinations == k  )
%                 data_mean.P( k, k   ) =       BinMean( k ) / n_frame;
%                 data_mean.P( k, k+1 ) = 1.0 - BinMean( k ) / n_frame;
%             else
%                 data_mean.P( k, k   ) = 0;
%                 data_mean.P( k, k+1 ) = 1.0;
%             end
%         end
%     else
        comb( combinations ) = 1;
        WBinMean = diag( comb ) * BinMean / n_frame;
        data_mean.P( lDiag ) =       WBinMean;
        data_mean.P( uDiag ) = 1.0 - WBinMean(1:end-1);
        data_mean.P( end,end ) = 1.0;
        
%         for k=1:BinCount
%             if any( combinations == k  )
%                 data_mean.P( k, k   ) =       BinMean( k ) / n_frame;
%                 data_mean.P( k, k+1 ) = 1.0 - BinMean( k ) / n_frame;
%             else
%                 data_mean.P( k, k   ) = 0;
%                 data_mean.P( k, k+1 ) = 1.0;
%             end
%         end
%     end

    data_mean.Q = data_mean.P(1:BinCount-1, 1:BinCount-1);
    data_mean.N = pinv( eye( BinCount-1 ) - data_mean.Q);
    data_mean.R = data_mean.P ( 1:BinCount-1, BinCount );
    data_mean.B = data_mean.N * data_mean.R;
    delay_mean  = zeros(BinCount, 1);
    for k=1:BinCount-1
        Tstop_human_mean  = data_mean.P(k,k) * Thuman;
        if Tstop_human_mean > Tstop_robot 
            delay_mean( k ) = Tstop_human_mean;
        elseif Tstop_human_mean > 0 
            delay_mean( k ) = Tstop_robot;
        else
            delay_mean( k ) = 0;
        end
    end
    delay_mean ( end ) = BinNSamples(end)/sum(BinNSamples(end)) * ( BinMean( end ) / n_frame ) * Thuman;

%    ret = Tnom + sum( delay_mean );
%    d = ( eye( BinCount-1 ) - data_mean.Q ) \ delay_mean(1:end-1) - delay_mean(1:end-1);
    ret = Tnom + ( data_mean.N(1,:) - 1 ) * delay_mean(1:end-1) + delay_mean(end);
%     ret = Tnom + d(1) + delay_mean(end);
    
end
