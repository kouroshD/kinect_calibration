clc; clear;
%% in this code we apply an iterative method for finding the Transformation Matrix from Kinect Reference tilted frame to Kinect Link which is unknown.

%_KrefUntilt_M8:        from kinect reference untilted frame to marker_8 frame
%_M8_Klink:             from Marker_8 frame to kinect link frame
%_KrefTilt_KrefUntilt:  from kinect reference tilted frame to kinect reference untilted frame 
format long;

t_KrefUntilt_M8=[0.936, 0.367, -0.873]';% read data from camera_left, 
q_KrefUntilt_M8=[-0.076, -0.078, 0.862, -0.4955];% read data from camera_left

q_KrefUntilt_M8=q_KrefUntilt_M8/norm(q_KrefUntilt_M8);
T_KrefUntilt_M8=quaternionToHomogenuesTrasnformation_kou(t_KrefUntilt_M8,q_KrefUntilt_M8);

t_M8_Klink=[0.5885, -0.674, 0.994]';% read data from kienct
q_M8_Klink=[-0.4265, 0.126, 0.7525, 0.486];% read data from kinect

q_M8_Klink=q_M8_Klink/norm(q_M8_Klink);
T_M8_Klink=quaternionToHomogenuesTrasnformation_kou(t_M8_Klink,q_M8_Klink);

T_KrefUntilt_Klink=T_KrefUntilt_M8*T_M8_Klink;

tilt=-50*pi/180; % we assume that the rotation is around y/g axis. 
% it should rotate -|tilt| angle in y direction
T_KrefTilt_KrefUntilt= [cos(tilt), 0, sin(tilt), 0; 0, 1, 0, 0; -sin(tilt), 0, cos(tilt), 0; 0, 0, 0, 1];

T_KrefTilt_Klink=T_KrefTilt_KrefUntilt*T_KrefUntilt_Klink;

[t_KrefTilt_Klink,q_KrefTilt_Klink]=homegenousTransformToQuaternion_kou(T_KrefTilt_Klink);

%% apply the new rotation matrix found previous section to find rotation matrix from head camera frame to Kinect untilted frame
new_tilt=-50*pi/180;

t_HC_M8=[0.430, -0.581, 1.151]';% read data from camera_left, 
q_HC_M8=[-0.243, 0.759, 0.5995, 0.069];% read data from camera_left

q_HC_M8=q_HC_M8/norm(q_HC_M8);
T_HC_M8=quaternionToHomogenuesTrasnformation_kou(t_HC_M8,q_HC_M8);


t_M8_Klink=[0.5885, -0.674, 0.994]';% read data from kienct
q_M8_Klink=[-0.4265, 0.126, 0.7525, 0.486];% read data from kinect

q_M8_Klink=q_M8_Klink/norm(q_M8_Klink);
T_M8_Klink=quaternionToHomogenuesTrasnformation_kou(t_M8_Klink,q_M8_Klink);

T_Klink_KrefTilt= homegenousTransInverse(T_KrefTilt_Klink);
T_KrefTilt_KrefUntilt= [cos(new_tilt), 0, sin(new_tilt), 0; 0, 1, 0, 0; -sin(new_tilt), 0, cos(new_tilt), 0; 0, 0, 0, 1];;

T_Klink_KrefUntilt=T_Klink_KrefTilt*T_KrefTilt_KrefUntilt;

T_HC_KrefUntilt=T_HC_M8*T_M8_Klink*T_Klink_KrefUntilt;
    
[t_HC_KrefUntilt,q_HC_KrefUntilt]=homegenousTransformToQuaternion_kou(T_HC_KrefUntilt);

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

