clc; clear;
%% in this code we apply Transformation Matrix from Kinect Reference tilted frame to Kinect Link which is unknown.
% we so this procedure for a couple of points in space and then make average.

%_KrefUntilt_B:        from kinect reference untilted frame to marker_8 frame
%_ball_Klink:             from ball frame to kinect link frame
%_KrefTilt_KrefUntilt:  from kinect reference tilted frame to kinect reference untilted frame 

% X_o X_w
format long;
id= fopen(strcat('points.txt')); 
structure=textscan(id,'%f %f %f %f %f %f');fclose(id);
% X_pcl X_baxter_gripper

X_pcl=[ structure{1},structure{2},structure{3}];
X_gripper=[structure{4},structure{5},structure{6}]; 
Number_of_points=length(X_pcl(:,1));


for i=1:Number_of_points
    t_world_ball_gripper=X_gripper(i,:)';
    q_world_ball=[0 0 0 1];
    T_world_ball_gripper=quaternionToHomogenuesTrasnformation_kou(t_world_ball_gripper,q_world_ball);

    t_world_ball_pcl=X_pcl(i,:)';
    q_world_ball=[0 0 0 1];
    T_world_ball_pcl=quaternionToHomogenuesTrasnformation_kou(t_world_ball_pcl,q_world_ball);
    T_ball_world_pcl=homegenousTransInverse(T_world_ball_pcl);

    t_KrefUntilt_world=[-0.299, 0.000, -0.788]';% read data from baxter gripper data 
    q_KrefUntilt_world=[0.014, 0.028, -0.031, 0.999];% read data from baxter gripper data 
    q_KrefUntilt_world=q_KrefUntilt_world/norm(q_KrefUntilt_world);
    T_KrefUntilt_world=quaternionToHomogenuesTrasnformation_kou(t_KrefUntilt_world,q_KrefUntilt_world);

    t_world_Klink=[0.258, 0.038, 0.818]';% read data from kienct pcl
    q_world_Klink=[-0.032, 0.384, 0.019, 0.922];% read data from kinect pcl
    q_world_Klink=q_world_Klink/norm(q_world_Klink);
    T_world_Klink=quaternionToHomogenuesTrasnformation_kou(t_world_Klink,q_world_Klink);

    T_KrefUntilt_ball=T_KrefUntilt_world*T_world_ball_gripper;
    T_ball_Klink=T_ball_world_pcl*T_world_Klink;


    T_KrefUntilt_Klink=T_KrefUntilt_ball*T_ball_Klink;


    tilt=-50*pi/180; % we assume that the rotation is around y/g axis. 
    % it should rotate -|tilt| angle in y direction
    T_KrefTilt_KrefUntilt= [cos(tilt), 0, sin(tilt), 0; 0, 1, 0, 0; -sin(tilt), 0, cos(tilt), 0; 0, 0, 0, 1];

    T_KrefTilt_Klink=T_KrefTilt_KrefUntilt*T_KrefUntilt_Klink;

    [t_KrefTilt_Klink,q_KrefTilt_Klink]=homegenousTransformToQuaternion_kou(T_KrefTilt_Klink);
    t_KrefTilt_Klink_vector(i,:)=t_KrefTilt_Klink';
    q_KrefTilt_Klink_vector(i,:)=q_KrefTilt_Klink;
end

t_KrefTilt_Klink=mean(t_KrefTilt_Klink_vector)';
q_KrefTilt_Klink=mean(q_KrefTilt_Klink_vector);


%% apply the new rotation matrix found previous section to find rotation matrix from head camera frame to Kinect untilted frame
new_tilt=-50*pi/180;
for i=1:Number_of_points

    t_HC_world=[0.001, -0.771, -0.051]';% read data from baxter gripper data 
    q_HC_world=[0.540, 0.546, 0.455, -0.451];% read data from baxter gripper data 
    q_HC_world=q_HC_world/norm(q_HC_world);
    T_HC_world=quaternionToHomogenuesTrasnformation_kou(t_HC_world,q_HC_world);

    t_world_ball_gripper=X_gripper(i,:)'; % read data from baxter gripper data 
    q_world_ball=[0 0 0 1];
    T_world_ball_gripper=quaternionToHomogenuesTrasnformation_kou(t_world_ball_gripper,q_world_ball);

    T_HC_ball=T_HC_world*T_world_ball_gripper;

    
    t_world_ball_pcl=X_pcl(i,:)';% read data from kinect pcl
    q_world_ball=[0 0 0 1];% read data from kinect pcl
    T_world_ball_pcl=quaternionToHomogenuesTrasnformation_kou(t_world_ball_pcl,q_world_ball);
    T_ball_world_pcl=homegenousTransInverse(T_world_ball_pcl);

    t_world_Klink=[0.258, 0.038, 0.818]';
    q_world_Klink=[-0.032, 0.384, 0.019, 0.922];
    q_world_Klink=q_world_Klink/norm(q_world_Klink);
    T_world_Klink=quaternionToHomogenuesTrasnformation_kou(t_world_Klink,q_world_Klink);

    T_ball_Klink=T_ball_world_pcl*T_world_Klink;


    T_Klink_KrefTilt= homegenousTransInverse(T_KrefTilt_Klink);
    T_KrefTilt_KrefUntilt= [cos(new_tilt), 0, sin(new_tilt), 0; 0, 1, 0, 0; -sin(new_tilt), 0, cos(new_tilt), 0; 0, 0, 0, 1];;

    T_Klink_KrefUntilt=T_Klink_KrefTilt*T_KrefTilt_KrefUntilt;

    T_HC_KrefUntilt=T_HC_ball*T_ball_Klink*T_Klink_KrefUntilt;
    
    [t_HC_KrefUntilt,q_HC_KrefUntilt]=homegenousTransformToQuaternion_kou(T_HC_KrefUntilt);
    
    t_HC_KrefUntilt_vector(i,:)=t_HC_KrefUntilt';
    q_HC_KrefUntilt_vector(i,:)=q_HC_KrefUntilt;
end
t_HC_KrefUntilt=mean(t_HC_KrefUntilt_vector)';
q_HC_KrefUntilt=mean(q_HC_KrefUntilt_vector);


%%%%%%%
T_KrefUntilt_KrefTilt=homegenousTransInverse(T_KrefTilt_KrefUntilt);
[t_KrefUntilt_KrefTilt,q_KrefUntilt_KrefTilt]=homegenousTransformToQuaternion_kou(T_KrefUntilt_KrefTilt);

% disp('HC_KrefUntilt')
disp(['rosrun tf static_transform_publisher ',num2str([t_HC_KrefUntilt', q_HC_KrefUntilt]),' /head_camera /KrefUntilt 100'])

% disp('KrefUntilt_KrefTilt')
disp(['rosrun tf static_transform_publisher ',num2str([t_KrefUntilt_KrefTilt', q_KrefUntilt_KrefTilt ]),' /KrefUntilt /KrefTilt 100' ])

% disp('KrefTilt_Klink')
disp(['rosrun tf static_transform_publisher ',num2str([t_KrefTilt_Klink' , q_KrefTilt_Klink]),'  /KrefTilt /cameraB_link 100' ])





%%%%%%%%%%%%%%%%%%%
% ITERATION 1
% rosrun tf static_transform_publisher 0.036472    0.061208    0.053826    -0.57526    -0.53535    -0.43277      0.4418 /head_camera /KrefUntilt 100
% rosrun tf static_transform_publisher 0           0           0           0     0.42262           0     0.90631 /KrefUntilt /KrefTilt 100
% rosrun tf static_transform_publisher -0.0037325    0.016032    0.021664  -0.0056139   -0.013153  -0.0091981     0.99986  /KrefTilt /cameraB_link 100

% ITERATION 2
% rosrun tf static_transform_publisher 0.03487    0.060707    0.052999    -0.57538    -0.53549    -0.43222       0.442 /head_camera /KrefUntilt 100
% rosrun tf static_transform_publisher 0           0           0           0     0.42262           0     0.90631 /KrefUntilt /KrefTilt 100
% rosrun tf static_transform_publisher -0.0047896   -0.024094    0.025294     0.01076 -0.00012479    0.012317     0.99987  /KrefTilt /cameraB_link 100

% ITERATION 3
% rosrun tf static_transform_publisher -0.69869     0.11903      1.0564    -0.78545    -0.15414   -0.013571     0.59927 /head_camera /KrefUntilt 100
% rosrun tf static_transform_publisher 0           0           0           0     0.42262           0     0.90631 /KrefUntilt /KrefTilt 100




% ITERATION 4
% rosrun tf static_transform_publishe0.03487    0.060707    0.052999    -0.57538    -0.53549    -0.43222       0.44227 /head_camera /KrefUntilt 100
% rosrun tf static_transform_publisher 0           0           0           0     0.42262           0     0.90631 /KrefUntilt /KrefTilt 100
% rosrun tf static_transform_publisher -0.0047896   -0.024094    0.025294     0.01076 -0.00012479    0.012317     0.99987  /KrefTilt /cameraB_link 100


% ITERATION 5
% rosrun tf static_transform_publisher 0.036903    0.063675    0.054286    -0.57545    -0.53468    -0.43323     0.44191 /head_camera /KrefUntilt 100
% rosrun tf static_transform_publisher 0           0           0           0     0.42262           0     0.90631 /KrefUntilt /KrefTilt 100
% rosrun tf static_transform_publisher -0.0072641  -0.0019935    0.014276   -0.005059   -0.013141  -0.0059165     0.99988  /KrefTilt /cameraB_link 100



%%%%%%%%%%%%%%%%%%%%%

% I found it based on on calibration iteration 5:

% Calibration based on the pcl:
% rosrun tf static_transform_publisher 0.03456    0.076452    0.063709    -0.57542    -0.53495    -0.43307     0.44179 /head_camera /KrefUntilt 100
% rosrun tf static_transform_publisher 0           0           0           0     0.42262           0     0.90631 /KrefUntilt /KrefTilt 100
% rosrun tf static_transform_publisher 0.017222     0.02564   0.0052574  -0.0045991   -0.013437  -0.0058198     0.99988  /KrefTilt /cameraB_link 100
% 
% it is working precise enough.






