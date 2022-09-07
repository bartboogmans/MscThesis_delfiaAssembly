classdef timerclass <handle
    %timerclass Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        a
        b
        timer1
        timer2
    end
    
    methods
        function obj = timerclass()
            obj.a = 1;
            obj.b = 2;
            
            obj.timer1 = timer(...
                'ExecutionMode', 'fixedRate', ...
                'Period', (1/2), ...
                'BusyMode', 'drop',...
                'TimerFcn', {@obj.timedFNC1},...
                'Name', 'timer1');
            
             obj.timer2 = timer(...
                'ExecutionMode', 'fixedRate', ...
                'Period', (1), ...
                'BusyMode', 'drop',...
                'TimerFcn', {@obj.timedFNC2},...
                'Name', 'timer2');
            
            obj.start();
        end
        
        function [] = timedFNC1(obj,~,~)
            %METHOD1 Summary of this method goes here
            %   Do some action that progresses the experiment in steps
            disp('timerfnc1');
            disp(obj.a)
        end
        
        function [] = timedFNC2(obj,~,~)
            %METHOD1 Summary of this method goes here
            %   Do some action that progresses the experiment in steps
            disp('timerfnc2');
            disp(obj.b);
            obj.b = obj.b +1;
        end
        
        
        function [] = start(obj)
           start(obj.timer1);
           start(obj.timer2);
        end
    end
end

