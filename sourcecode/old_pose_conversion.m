            %% Old code in case I fucked up the script called in the line above. [bb]
            %%{ 
            obj.x(1) = message.Pose.Position.Z; % Horizontal, in line with the tank (positive = away from carriage)
            obj.x(2) = message.Pose.Position.X; % Horizontal, perpendicular to length of tank (positive = towards walkway)
            %eul = quat2eul([message.Pose.Orientation.X,message.Pose.Orientation.Y,message.Pose.Orientation.Z ,message.Pose.Orientation.W]);
            eul = euler(quaternion([message.Pose.Orientation.W,message.Pose.Orientation.X,message.Pose.Orientation.Y ,message.Pose.Orientation.Z]),'YXZ','frame');
            %eul
            obj.x(3) = eul(1);
            %disp('ros_sub_pose');
            %%}
            