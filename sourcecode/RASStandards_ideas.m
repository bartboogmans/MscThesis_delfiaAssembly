function settings = RASStandards()

optitrack_pc_IP = 'http://192.168.1.1:11311';

settings.msgtypes.actuation     =       'std_msgs/Float64MultiArray';
settings.msgtypes.reference     =       'geometry_msgs/Pose2D';
settings.msgtypes.pose          =       'geometry_msgs/PoseStamped';



settings.topicnames = cell(num_vessels,3);
for i = 1:num_vessels
    settings.topicnames{i,1} = join(['/ActuationDelfia',num2str(i)]);
    settings.topicnames{i,2} = join(['/ReferenceDelfia',num2str(i)]);
    settings.topicnames{i,3} = join(['/vrpn_client_node/Delfia',num2str(i),'/pose']);
end



ROS:
IP Adresses
- 10 Delfia
- x GreySeabax
- y TitoNeri
- ROS Master
- define open channels?


topics:
- pose 
    type
    name
- actuation
    type
    name
- other: (reference?)
    type
    name

Delfia: Serial communication protocol