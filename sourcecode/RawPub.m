classdef RawPub < handle
    %RawPub Publishes raw pose data on a ros topic at a specified interval
    % The 
    
    properties
        node
        pub
        
        
        f_data
        f_i
        s_data
        s_i
        nlog
        
        time
        timer
        t_end
        f_t_last
        nlog_index
        
        sendtimestart
    end
    
    methods
        function obj = RawPub()
            
            settings = delfiaSettings();
            obj.node = ros.Node('rawpubnode',settings.hostname,'NodeHost',settings.myIP);
            obj.pub = ros.Publisher(obj.node, settings.topicnames.vesselPose{1},settings.messageTypes.vesselPose);
            obj.nlog_index = 1;
            obj.f_data = zeros(2,10000);
            obj.f_i = 1;
            obj.s_data = zeros(2,10000);
            obj.s_i = 1;
            obj.f_t_last = 0;
            obj.nlog = 3;
            obj.t_end = 8;
            obj.sendtimestart = 0;
            
            obj.timer = timer(...
                'ExecutionMode', 'fixedRate', ...           
                'Period', 1/400, ...                 
                'BusyMode', 'drop',...                    
                'TimerFcn', {@obj.timerfnc},...
                'Name', 'rawpubtimer' );
        end
        
        
        function obj = timerfnc(obj,~,~)
            
            
            if obj.nlog_index >= obj.nlog  
                obj.sendtimestart = tic;
            end
            
            % Send an empty message
            msg = rosmessage(obj.pub);
            send(obj.pub,msg)
            
            % store only data every nlog iterations, to not affect
            % publishrate by this observing code
            if obj.nlog_index >= obj.nlog  
                
                % Log the last message time
                dt_pub = toc(obj.sendtimestart);         
                t = toc(obj.time);
                obj.s_data(:,obj.s_i) = [t;dt_pub];
                obj.s_i = 1+obj.s_i;
                
                % Log the average message time since last log
                dt = t-obj.f_t_last;
                obj.f_data(:,obj.f_i) = [t;dt];
                obj.f_t_last = t;
                obj.f_i = 1+obj.f_i;
                
                obj.nlog_index = 0;
                
                % Stop the experiment
                if t>obj.t_end
                    obj.stop();
                    obj.plotFreq();
                end
            end 
            
            obj.nlog_index = 1 + obj.nlog_index;
        end
        
        function [] = plotFreq(obj)
            
            figure
            t = obj.f_data(1,1:obj.f_i-1);
            dtn = (obj.f_data(2,1:obj.f_i-1))/obj.nlog;
            
            subplot(2,2,3)
            plot(t,dtn);
            xlabel('t [seconds]')
            ylabel('freq [hz]')
            title('Averaged pubtime over time');
            
            f = zeros(1,length(dtn));
            for i = 1:length(dtn)
                f(i) = (1/dtn(i));
            end
            
            subplot(2,2,1)
            plot(t,f);
            xlabel('Time [seconds]')
            ylabel('Frequency [hz]')
            title('Frequency broadcast over time');
            
            subplot(2,2,2)
            t2 = obj.s_data(1,1:obj.s_i-1);
            dt1 = obj.s_data(2,1:obj.s_i-1);
            plot(t2,dt1);
            xlabel('t [seconds]')
            ylabel('publishtime [seconds]')
            title('publish speed over time');
        end
        
        function [] = start(obj)
            obj.time = tic;
            % Check for existing node
            if isempty(obj.node)
                try
                    settings = delfiaSettings();
                    obj.node = ros.Node('rawpubnode',settings.hostname,'NodeHost',settings.myIP);
                    disp('succesfully re started the node');
                catch
                    disp('Warning: no obj.node detected and re initiation failed');
                end
            end
            
            % Sart the timer
            try
                start(obj.timer);
            catch
                disp('Error during starting of timer');
                disp(join(['isempty(timer) = ',string(isempty(obj.timer))]));
            end
        end
        
        function [] = stop(obj)
            
            % Stop the timer
            try
                stop(obj.timer);
            catch
                disp('Error during stopping of timer');
                disp(join(['isempty(obj.timer) = ',string(isempty(obj.timer))]));
            end
            
            % Delete the timer
            try
                delete(obj.timer);
            catch
                disp('Error during deletion of timer');
                disp(join(['isempty(obj.timer) = ',string(isempty(obj.timer))]));
            end
            
            
            % Delete the node
            try
                delete(obj.node);
            catch
                disp('Error during deleting of node');
                disp(join(['isempty(obj.node) = ',string(isempty(obj.node))]));
            end
            
        end
    end
end

