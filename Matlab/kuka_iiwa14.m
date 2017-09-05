%% DH transformation matrices and direct kinematics of a serial robot (IIWA14 KUKA)
%% 3 Oct 2016 (Pellegrinelli)


%% Initialize the robotics toolbox robot
clear robot L
%L(1)=Link([theta,d,a,alpha,0 for revolutel 1 for prismatic,offset])
L(1)=Link('d',0.36,'a',0.0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-170);deg2rad(170)],'revolute');   
L(2)=Link('d',0.00,'a',0.0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-120);deg2rad(120)],'revolute');   
L(3)=Link('d',0.42,'a',0.0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-170);deg2rad(170)],'revolute');   
L(4)=Link('d',0.00,'a',0.0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-120);deg2rad(120)],'revolute');   
L(5)=Link('d',0.40,'a',0.0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-170);deg2rad(170)],'revolute');   
L(6)=Link('d',0.00,'a',0.0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-120);deg2rad(120)],'revolute');  
L(7)=Link('d',0.126,'a',0.0,'alpha',0,'offset',0,'qlim',[deg2rad(-175);deg2rad(175)],'revolute');  

%Mworldjoint = rpy2tr(0,0,1.57);
Mworldjoint=eye(4);

% Meejrobotiqadapter = transl(0, 0, 0.063) * rpy2tr( 0.82, -1.5707963, 0.0 );
% Mrobotiq85basejoint = transl(0, 0, 0) * rpy2tr( .025, -0.01, -0.0034 );
% Meejroboticcontrol = transl(0.12, 0., -0.01 )*rpy2tr( 0.0, 1.5707963, 0.0 );
% Mtool = Meejrobotiqadapter*Mrobotiq85basejoint*Meejroboticcontrol;

Mtool = transl(0.016, 0.003, 0.208)*rpy2tr(0.000, -0.000, 0.820);

robot=SerialLink(L,'name','iiwa14','base',Mworldjoint,'tool',Mtool); 


% q2=[1.2 0.3 0.0 0.1 0.1 0.4 0.7]
% robot.plot(q2)
% robot.fkine(q2)


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



%% end
