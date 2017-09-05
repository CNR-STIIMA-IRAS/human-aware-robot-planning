function theta = ikine6sMOD(robot, Tbt, qseed)
    
    if ~ishomog(Tbt)
        error('RTB:ikine:badarg', 'Tbt is not a homog xform');
    end
    
    L = robot.links;
    
    if ~robot.isspherical()
        error('RTB:ikine:notsupported', 'wrist is not spherical');
    end
    
    if isempty( robot.qlim )
        robot.qlim = ones( robot.n, 2 );
        robot.qlim(:,1) = -pi; 
        robot.qlim(:,2) =  pi; 
    end
    
    % undo base and tool transformations
    T06 = inv(robot.base) * Tbt * inv(robot.tool);
            
        
    %% now solve for the first 3 joints, based on position of the spherical wrist centre

    % general case with 6 length parameters
    Tw = T06(1:3,4) - T06(1:3,3) * L(6).d;
    Tx = Tw(1);     Ty = Tw(2);   Tz = Tw(3);
    
    Dw = Tw - [ 0;0; L(1).d ];
    dw = sqrt( Dw' * Dw );

    %%%%%%%%%%%%%%%%%%%%% q1
    q1 = [ atan2(Ty, Tx); 
           pi + atan2(Ty, Tx); ];
    q1 = sortSol( q1, qseed(1), L(1).offset, robot.qlim(1,:), 'q1' );
    %%%%%%%%%%%%%%%%%%%%% q1 end
        
    %%%%%%%%%%%%%%%%%%%%% q3 
    
    c3  = (dw^2 - L(2).a^2 - L(4).d^2)/(2*L(2).a*L(4).d );
    sol = [];
    if c3>=-1 && c3<=1   
    
        q3  = [ atan2(  sqrt(1-c3^2), c3 );
                atan2( -sqrt(1-c3^2), c3 );]; 
            %%% q3 = 0 means that the system is aligned! 
            %%% Therefore, the offset has to be superimposed to zero
        q3 = sortSol( q3, qseed(3), 0.0, robot.qlim(3,:), 'q3' );
        %q3 = [q3;];

        %%%%%%%%%%%%%%%%%%%%% q2
            q2_ = [];

        q2a = [ asin( sqrt( Dw(1)^2+Dw(2)^2 ) / dw ); 
               pi + asin( sqrt( Dw(1)^2+Dw(2)^2 ) / dw ) ];
        q2b = [ acos( ( dw^2 + L(2).a^2 - L(4).d^2)/(2*L(2).a*dw ) );
               -acos( ( dw^2 + L(2).a^2 - L(4).d^2)/(2*L(2).a*dw ) );];
        ok = 0;
        for i=1:length(q3)
            for j=1:length(q2b)
                for k=1:length(q2a)
                    if  consistencyCheck( q2b(j)+q2a(k), q3(i), Dw, L(2).a, L(4).d )
                        q2_ = [q2_; q2b(j)+q2a(k)];
                        ok = 1;
                    elseif consistencyCheck( q2b(j)-q2a(k), q3(i), Dw, L(2).a, L(4).d )
                        q2_ = [q2_; q2b(j)-q2a(k)];
                        ok = 1;
                    elseif consistencyCheck( -q2b(j)+q2a(k), q3(i), Dw, L(2).a, L(4).d )
                        q2_ = [q2_; -q2b(j)+q2a(k)];
                        ok = 1;
                    elseif consistencyCheck( -q2b(j)-q2a(k), q3(i), Dw, L(2).a, L(4).d )
                        q2_ = [q2_; -q2b(j)-q2a(k)];
                        ok = 1;
                    end
                end
            end
        end
        q2_ = [ q2_; -q2_ ];
        if ok == 0
            theta = []; 
            return 
        end

        %%% q2 = 0 means that the system is aligned! 
        %%% Therefore, the offset has to be superimposed to zero
        q2 = sortSol( q2_, qseed(2), 0, robot.qlim(2,:), 'q2' );

        clear L
        L(1)=Link('d',0.360,'a',0.000,'alpha', pi/2,'offset',0.00,'qlim',[deg2rad(-170);deg2rad(170)],'revolute');   
        L(2)=Link('d',0.000,'a',0.420,'alpha',   pi,'offset',pi/2,'qlim',[deg2rad(-120);deg2rad(120)],'revolute');   
        L(3)=Link('d',0.000,'a',0.000,'alpha', pi/2,'offset',pi/2,'qlim',[deg2rad(-120);deg2rad(120)],'revolute');   

        rr = SerialLink(L,'name','iiwa14_3dof','base',eye(4),'tool',eye(4)); 

        sol = [];
        q4_ = [];
        q5_ = [];
        q6_ = [];
        for i=1:length(q1)
            for j=1:length(q2)
                for k=1:length(q3)
                    T03 = rr.fkine( [q1(i), q2(j),q3(k) ] );
                    T36 = (T03^-1)* T06;

                    q4a = atan2( T36(2,3),T36(1,3) ); 
                    q5a = atan2( sqrt( T36(2,3)^2+T36(1,3)^2), T36(3,3) ); 
                    q6a = atan2( T36(3,2),-T36(3,1) );
                    if q4a >= robot.qlim(4,1) && q4a <= robot.qlim(4,2 ) ...
                    && q5a >= robot.qlim(5,1) && q5a <= robot.qlim(5,2 ) ...
                    && q6a >= robot.qlim(6,1) && q6a <= robot.qlim(6,2 ) 
                        if max(max( robot.fkine( [q1(i), q2(j), q3(k), q4a, q5a, q6a] )* Tbt^-1 - eye(4) )) < 1e-5
                            sol = [ sol; q1(i), q2(j), q3(k), q4a, q5a, q6a ];
                        end
                    end

                    q4b = atan2( -T36(2,3),-T36(1,3) ); 
                    q5b = atan2( -sqrt( T36(2,3)^2+T36(1,3)^2), T36(3,3) );
                    q6b = atan2( -T36(3,2),T36(3,1) );
                    if q4b >= robot.qlim(4,1) && q4b <= robot.qlim(4,2 ) ...
                    && q5b >= robot.qlim(5,1) && q5b <= robot.qlim(5,2 ) ...
                    && q6b >= robot.qlim(6,1) && q6b <= robot.qlim(6,2 ) 
                        if max(max( robot.fkine( [q1(i), q2(j), q3(k), q4b, q5b, q6b] )* Tbt^-1 - eye(4) )) < 1e-5
                            sol = [ sol; q1(i), q2(j), q3(k), q4b, q5b, q6b ];
                        end
                    end
                end
            end
        end
    end
    
    if size(sol,1)==0
        theta=[];
        
    else
        theta = unique( sol, 'rows' );
    
        diff_from_seed = zeros(size(theta,1),1);
        for i=1:size(theta,1)
            diff_from_seed(i) = (theta(i,:)-qseed')*(theta(i,:)-qseed')';
        end
        [~,I]=sort(diff_from_seed);
        theta=theta(I,:);        
    end
    

end



function ok = consistencyCheck( q2, q3, Dw, L2, L4 )
    ok = ( (Dw(3) - L2*cos(q2)  - L4*cos(q2 + q3))^2 ... 
         + (sqrt(Dw(1)^2 + Dw(2)^2) - L2*sin(q2)  - L4*sin(q2 + q3))^2  ) ...
       < 1e-6;
end

function q = sortSol( qq, qseed, qoffset, qlim, hdr )
    
    q = [];
    
    [~,I] = unique( round(qq*1e3)/1e3 );
    qq = qq(I);
    [~,I] = sort( abs( qq(:) - (qseed+qoffset) ) );
    qq = qq(I);
    for i=1:length(qq)
        if qq(i) < qlim(1) + qoffset
            qq(i) = qq(i) + 2*pi;
        end
        if qq(i) > qlim(2) + qoffset
            qq(i) = qq(i) - 2*pi;
        end
        if qq(i) >= qlim(1)+ qoffset && qq(i) <= qlim(2) + qoffset
            q = [q;qq(i);];
        end
    end
    
%     for i=1:length(q)
%         disp( [ hdr, ' seed:', num2str( qseed + qoffset), ' sol: ' num2str( q(i) ), ' diff: ', num2str( qseed + qoffset - q(i) )  ] );
%     end

end
    