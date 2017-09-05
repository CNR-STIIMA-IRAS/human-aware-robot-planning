% clear all
% close all
% clc

%% DH transformation matrices and direct kinematics of a serial robot (IIWA14 KUKA)
%% 3 Oct 2016 (Pellegrinelli)


%% Initialize the robotics toolbox robot

L(1)=Link('d',0.360,'a',0.000,'alpha', pi/2,'offset',0.00,'qlim',[deg2rad(-170);deg2rad(170)],'revolute');   
L(2)=Link('d',0.000,'a',0.420,'alpha',   pi,'offset',pi/2,'qlim',[deg2rad(-120);deg2rad(120)],'revolute');   
L(3)=Link('d',0.000,'a',0.000,'alpha', pi/2,'offset',pi/2,'qlim',[deg2rad(-120);deg2rad(120)],'revolute');   
L(4)=Link('d',0.400,'a',0.000,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-170);deg2rad(170)],'revolute');   
L(5)=Link('d',0.000,'a',0.000,'alpha', pi/2,'offset',0,'qlim',[deg2rad(-120);deg2rad(120)],'revolute');  
L(6)=Link('d',0.126,'a',0.000,'alpha',0,'offset',0,'qlim',[deg2rad(-175);deg2rad(175)],'revolute');  

Mworldjoint = eye(4); % rpy2tr(0,0,1.57);
%Mworldjoint(1:3,4)=[0.5 -0.8 0];
%Mtool       = eye(4); 
Mtool = transl(0.016, 0.003, 0.208)*rpy2tr(0.000, -0.000, 0.820);

robot=SerialLink(L,'name','iiwa14_mod','base',Mworldjoint,'tool',Mtool); 

% close all
% figure
% %q1=[0.1 0.15 1.1 0 0.45 0.7];
% q1 = robot.qlim(:,1 ) + diag( rand(6,1) ) * ( robot.qlim(:,2 )- robot.qlim(:,1 ) );
% robot.plot(q1')
% robot.fkine(q1)

% lagrange@lagrange:~/ros_euroc_ws$ rosrun tf tf_echo world ee_tool 
% At time 1495206522.492
% - Translation: [-0.003, 0.016, 1.514]
% - Rotation: in Quaternion [0.000, 0.000, 0.930, 0.367]
%             in RPY (radian) [0.000, -0.000, 2.390]
%             in RPY (degree) [0.000, -0.000, 136.937]

% lagrange@lagrange:~/ros_euroc_ws$ rosrun tf tf_echo ee_link ee_tool 
% At time 1495206750.080
% - Translation: [0.016, 0.003, 0.208]
% - Rotation: in Quaternion [0.000, -0.000, 0.399, 0.917]
%             in RPY (radian) [0.000, -0.000, 0.820]
%             in RPY (degree) [0.000, -0.000, 46.983]

% qq = robot.qlim(:,1 ) + diag( rand(6,1) ) * ( robot.qlim(:,2 )- robot.qlim(:,1 ) );
% disp([ 'q: ', num2str( q1' ) ] );
% ee = robot.fkine(q1); 
% 
% qq = ikine6sMOD( robot, ee, q1 ); 
% for i=1:size(qq,1)
%     disp([num2str(i),'# q: ', num2str( qq(i,:) ) , ' check T err: ', num2str( max(max( robot.fkine(qq(i,:) )* ee^-1 - eye(4) )) ),  '  Dq err: ', num2str( q1'-qq(i,:) ) ] );
% end


%% end
