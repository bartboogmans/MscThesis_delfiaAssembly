classdef ImpulseTestController <handle
    %UNTITLED Summary of this class goes here
    %   Used for experiments on the 1rst of april 2021
    
    properties
        sendtimer
        sendFrequency
        fleet
        controllers 
        phase
        tictime
    end
    
    methods
        function obj = ImpulseTestController()
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            settings = delfiaSettings();
            
            obj.phase = 0;
            obj.tictime = tic;
            
            obj.fleet = Delfia.empty;
            for i = 1:settings.n_vessels
                obj.fleet(i) = Delfia(0,0,0,settings.vesselnames{i});
            end
            
            obj.controllers = MultiVesselPlatformController.empty;
            for i = 1:settings.n_platform_controllers
                obj.controllers(i) = MultiVesselPlatformController(0,0,0,join(['impulseTestController',string(i)],'_'));
                obj.controllers(i).initROSNode();
                obj.controllers(i).attachBody(obj.fleet(i)); % This sets up an unwanted control callback aswell. see below.
                delete(obj.fleet(i).location_subscriber); % Disable controlling callback of the controller object.
            end
            
            obj.sendtimer = timer('ExecutionMode','fixedRate','Period',5,'BusyMode', 'drop','TimerFcn',{@obj.timerFcn});
            start(obj.sendtimer)
        end
        
        
        function  timerFcn(obj,~,~)
            %timerFcn timed broadcasting of platform actuation
            %   Called by obj.sendtimer timer object at the rate of obj.sendFrequency
            switch obj.phase
                case 0
                    for vessel = obj.fleet
                        vessel.thrSpd = [40;40];
                        vessel.thrAngle = [pi/2;pi/2];
                        vessel.ros_pub_actuation();
                    end
                    
                    if toc(obj.tictime)>2
                        obj.phase = obj.phase+1;
                        obj.tictime = tic;
                    end
                    
                case 1
                    for vessel = obj.fleet
                        vessel.thrSpd = [0;0];
                        vessel.thrAngle = [pi/2;pi/2];
                        vessel.ros_pub_actuation();
                    end
                    
                    if toc(obj.tictime)>2
                        obj.phase = obj.phase+1;
                        obj.tictime = tic;
                    end
                    
                otherwise
                    disp('deleting test object');
                    delete(obj)
            end
        end
        

        function delete(obj)
            if ~isempty(obj.fleet)
                delete(obj.fleet)
            end
            if ~isempty(obj.controllers)
                delete(obj.controllers)
            end
            if ~isempty(obj.sendtimer)
                stop(obj.sendtimer)
                delete(obj.sendtimer)
            end
        end
    end
end

