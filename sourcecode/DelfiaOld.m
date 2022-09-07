classdef DelfiaOld < Vessel & handle & matlab.mixin.CustomDisplay
    properties
        
        % Actuator states
        thrAngle                    % [radians]     (1x2)   Angles of thrusters
        thrForce                    % [N]           (1x2)   Estimated propulsion force of a thruster (based on ref speed)
        thrSpd                      % [rounds/s]    (1x2)   Reference speed of thrusters
        
        % Body states
        platf_eta                   % [m,m,radian]  (1x3)   Pose of this vessel in platform frame if it is connected
        currentController           % [Handle]      (1x1)   Handle to controlling entity.

        % Communication
        actuation_publisher         % [ROS publisher]       Object for sending data to ROS topics
        actuation_subscriber        % [ROS subscriber]      Object for subscribing to ROS topics
        location_publisher
        location_subscriber
        reference_publisher
        reference_subscriber
        
        % Parameters
        r_thr                       % [m]           (2x2)   Location of thrusters in local frame
        RPSmin                      % [rounds/s]    (1x1)   Minimum speed of azimuth propeller
        RPSmax                      % [rounds/s]    (1x1)   Maximum speed of azimuth propeller
        thrFmax                     % [N]           (1x1)   Maximum force of single azimuth at full power
        I_z                         % [kg*m^2]      (1x1)   Estimated moment of inertia around z axis
                
        % Display
        plotThr                     % [Bool]        (1x1)   Boolean true if the thruster is to be plotted
        plotF                       % [Bool]        (1x1)   Boolean true if thruster forces are to be plotted
        forceColor                  % [rgb colormap](2x3)   Color of the force vectors from the thrusters
        refColor                    % [rgb colormap](1x3)   Color of the reference
        thrOutline                  % [double]      (2xi)   Array with data on the thruster perimeter
    end

    methods
        function obj = Delfia(x_,y_,yaw_,name_,~)
            
            obj = obj@Vessel();
            
            if nargin >= 2
                obj.x(1:3) = [x_;y_;yaw_];
            end
            if nargin == 4
                obj.name = name_;
            end
            
            
            obj.w = 0.20;
            obj.l = 0.38;
            obj.m = 4; %kg for calculating Centre of mass
            
            % Prepare Display
            cf = 0.015; % decorative chamfer
            obj.outline = [   obj.l/2      ,obj.l/2-cf   ,-obj.l/2+cf    ,-obj.l/2       ,-obj.l/2       ,-obj.l/2+cf    ,obj.l/2-cf     ,obj.l/2        ,obj.l/2 ;
                                    obj.w/2-cf   ,obj.w/2      ,obj.w/2        ,obj.w/2-cf     ,-obj.w/2+cf    ,-obj.w/2       ,-obj.w/2       ,-obj.w/2+cf    ,obj.w/2-cf];                   
            obj.thrOutline =   [   0.006,      -0.013,     -0.013,     -0.037,     -0.037,     -0.013,     -0.013,     0.006,      0.006;
                                        -0.0075,    -0.0075,    -0.023,     -0.020,     0.020,      0.023,      0.0075,     0.0075,     -0.0075 ];
            obj.r_thr = [-0.155,0;0.155,0];
            obj.thrAngle = [0,0];
            obj.thrForce = [0,0];
            obj.thrSpd = [0,0];
            obj.plotThr = true;
            obj.plotF = true;
%            obj.plotRef = true;
            obj.forceColor = [0,1,0 ; 1,0,0];
            obj.color = (1-obj.color)*0.7 + obj.color;
            
            
            %{
            % Prepare controls
            kp_xy = 40.00;
            ki_xy = 0.1;
            kd_xy = 0;
            kp_a = 0.6;
            ki_a = 0;
            kd_a = 0;
            obj.PIDx = dPID(kp_xy, ki_xy, kd_xy);
            obj.PIDy = dPID(kp_xy, ki_xy, kd_xy);
            obj.PID_yaw = dPID(kp_a, ki_a, kd_a);
            
            obj.t_last_update = 0;
            %}
            obj.thrFmax = 1.5;      % [N] Rough measurement. I estimate a possibility for 20% error, so use with caution. [BB]
            obj.I_z = 0.031 + 0.110;        % [kg * m^2] Source: Maurits' model. The first part is inertia from rigid body, the second is due to added mass. 
            obj.platf_eta = [0;0;0];
        end
        
        function obj = location_update_callback(obj,message)
            obj.x(1) = message.Pose.Position.Z; % Horizontal, in line with the tank (positive = away from carriage)
            obj.x(2) = -message.Pose.Position.X; % Horizontal, perpendicular to length of tank (positive = towards walkway)
            eul = quat2eul([ message.Pose.Orientation.W,message.Pose.Orientation.Z,-message.Pose.Orientation.X ,-message.Pose.Orientation.Y]);
            obj.x(3) = -eul(1); %% Check this bad boi. 
        end
        
        function obj = actuation_update_callback(obj,message)
            obj.thrSpd = [message.Data(1),message.Data(2)];
            obj.thrAngle = [message.Data(3),message.Data(4)];
            obj.calcThrForce;
        end
        
        function [] = publish_pose(obj,time)
            msg = rosmessage(obj.location_publisher);
            msg.Pose.Position.X = obj.x(1);
            msg.Pose.Position.Y = obj.x(2);
            msg.Pose.Position.Z = 0;
            
            quat = eul2quat([obj.x(3),0,0]);
            msg.Pose.Orientation.X = quat(1);
            msg.Pose.Orientation.Y = quat(2);
            msg.Pose.Orientation.Z = quat(3);
            msg.Pose.Orientation.W = quat(4);
            
            msg.Header.Stamp.Sec = floor(time);
            msg.Header.Stamp.Nsec = floor((time - floor(time))*1000000);
            
            send(obj.location_publisher,msg)
        end
                
        %{
        function obj = update_actuators(obj,systime)
            if length(obj.refx)==1 || length(obj.refy)==1 || length(obj.ref_yaw)==1
                
                dt = systime-obj.t_last_update;
                
                % calculate errors
                obj.recalculate_errors();
                
                % run controls to get forces/moments on body
                Fx = obj.PIDx.run(obj.ex,dt);
                Fy = obj.PIDy.run(obj.ey,dt);
                Mc = obj.PID_yaw.run(obj.e_yaw,dt);
                
                % Convert forces/moments to actuator state
                obj.force_to_actuator_states(Fx,Fy,Mc);
                
                % Send commands via ROS
                msgOut = rosmessage('std_msgs/Float64MultiArray');
                msgOut.Data = [obj.force2RPS(1),obj.force2RPS(2),obj.thrAngle(1),obj.thrAngle(2)];
                send(obj.actuation_publisher,msgOut);
            end
            obj.t_last_update =  systime;
        end
        %}
        
        %{
        function obj = force_to_actuator_states(obj,F,M)
            % convert to local frame
            FL = R2d(-obj.pose.yaw)*F; 
            
            % Calculate outgoing forces that the actuators should produce
            F_out = [(FL(1)/2),(FL(2)+M/obj.r_thr(1,1))/2 ; (FL(1)/2),(FL(2)+M/obj.r_thr(2,1))/2];
            
            % Calculate and set outgoing actuator state
            obj.thrForce = [sqrt(F_out(1,1)^2+F_out(1,2)^2),sqrt(F_out(2,1)^2+F_out(2,2)^2)];

            for i=1:2
                if obj.thrForce(i) > 0 % Only adjust thruster angle if it is active 
                    obj.thrusterAngle = [angle(F_out(1,1)+1i*F_out(1,2)),angle(F_out(2,1)+1i*F_out(2,2))];
                end
            end
        end
        %}
        
        % Sets thruster number i to actuate forcevector F
        % F is defined in local frame
        function obj = thrusterforce_to_actuation(obj,i,F)
            
            obj.thrForce(i) = sqrt(F(1)^2 + F(2)^2);
            
            if obj.thrForce(i) > 0 % Only adjust thruster angle if it is active 
                    obj.thrAngle(i) = angle(F(1)+1i*F(2));
            end
            obj.thrusterRPS = obj.force2RPS(obj,i);
            % Something with sending actuation via publisher
        end
  
        %{
        function obj = recalculate_errors(obj)
            obj.ex = obj.refx-obj.x;
            obj.ey = obj.refy-obj.y;
            obj.e_yaw = obj.ref_yaw-obj.pose.yaw - 2*pi*floor( (obj.ref_yaw-obj.pose.yaw+pi)/(2*pi) );
        end
        %}
        
        function obj = plotVessel(obj,ax)
            R = R2d(obj.x(3)); % rotation matrix for yaw around Z axis
            
            % Plot the hull
            plotVessel@Vessel(obj,ax);
            
            % Plot the thrusters
            if obj.plotThruster == true
                for i= 1:length(obj.r_thr)
                    thruster_vector = R*(R2d(obj.thrAngle(i))*obj.thrusterPerimeter + obj.r_thr(i,:)') + obj.pose.loc;
                    plot(ax,thruster_vector(1,:),thruster_vector(2,:),'color',obj.hullcolor);
                end
            end
                        
            % plot the force vectors assigned to be applied by the thrusters
            if obj.plotForces == true
                for i= 1:length(obj.r_thr)
                    force_vector = R*(0.1*R2d(obj.thrAngle(i))*[0,obj.thrForce(i);0,0] + obj.r_thr(i,:)') + obj.pose.loc;
                    plot(ax,force_vector(1,:),force_vector(2,:),'color',obj.forceColor(i,:))
                end
            end
            
            % plot the reference shadow of the vessel
            if obj.plotReference == true && length(obj.ref_yaw)==1 && length(obj.refx)==1 && length(obj.refy)==1
                R_ref = R2d(obj.ref_yaw);
                refHullVec = R_ref*obj.plotPerimeter + [obj.refx,obj.refy]';
                refArrowVec = R_ref*[-0.08,0.08,0.04,0.08,0.04;0,0,0.01,0,-0.01] + [obj.refx,obj.refy]';
                plot(ax,refHullVec(1,:),refHullVec(2,:),'color',obj.refColor);
                plot(ax,refArrowVec(1,:),refArrowVec(2,:),'color',obj.refColor);                
            end            
        end
        
        function rps = force2RPS(obj,i)
            % This is a temporary estimation only
            rps = obj.thrForce(i)*10;
        end
        
        function obj = calcThrF(obj)
            % convert RPS to force. This is an estimation only. Use with
            % care. [BB]
            for i = 1:length (obj.thrForce)
                obj.thrForce(i) = obj.thrSpd(i)/10;
            end
        end
    end
end