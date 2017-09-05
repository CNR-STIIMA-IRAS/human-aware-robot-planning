function ret = MC( BinCount, BinMean, BinNSamples, combinations, Thuman, Tstop_robot, Tnom, n_frame, lDiag, uDiag )
    
    comb  = zeros( BinCount, 1 );
    P     = eye( BinCount  , BinCount );  
    
    comb( combinations ) = 1;
    WBinMean     = diag( comb ) * BinMean / n_frame;
    P( lDiag )   =       WBinMean;
    P( uDiag )   = 1.0 - WBinMean(1:end-1);
    P( end,end ) = 1.0;
        
    Q = P(1:BinCount-1, 1:BinCount-1);
    
    %N = ( eye( BinCount-1 ) - Q)^-1;
    R = P ( 1:BinCount-1, BinCount );
    %B = N * R;
    delay_mean  = zeros(BinCount, 1);
    for k=1:BinCount-1
        Tstop_human_mean  = P(k,k) * Thuman;
        if Tstop_human_mean > Tstop_robot 
            delay_mean( k ) = Tstop_human_mean;
        elseif Tstop_human_mean > 0 
            delay_mean( k ) = Tstop_robot;
        else
            delay_mean( k ) = 0;
        end
    end
    delay_mean ( end ) = BinNSamples(end)/sum(BinNSamples(end)) * ( BinMean( end ) / n_frame ) * Thuman;


%   ret = Tnom + ( N(1,:) - 1 ) * delay_mean(1:end-1) + delay_mean(end);
    d = ( eye( BinCount-1 ) - Q ) \ delay_mean(1:end-1) - delay_mean(1:end-1);
    ret = Tnom + d(1) + delay_mean(end);
    
    
end
