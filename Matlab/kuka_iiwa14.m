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
Mworldjoint=eye(4);
Mtool = transl(0.016, 0.003, 0.208)*rpy2tr(0.000, -0.000, 0.820);
robot=SerialLink(L,'name','iiwa14','base',Mworldjoint,'tool',Mtool); 


%% end
