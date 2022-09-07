%% Ratemonitor: Monitors the rate of ros topics for evaluation.

% Usage:
% optional: edit your topic of interested at section 'TOPIC TO EVALUATE'
% Run this script.
% Stop this script by calling rateMonitor.stopFnc();
% evaluate rateMonitor.log
% restart the logging process by calling rateMonitor.startFnc();
clear rateMonitor
global rateMonitor
rateMonitor.stop = @stopFnc;
rateMonitor.start = @startFnc;
rateMonitor.plot = @plotLog;

%% TOPIC TO EVALUATE: EDIT HERE
settings = delfiaSettings;
rateMonitor.topicname = settings.topicnames.vesselPose{1};
rateMonitor.topictype = settings.messageTypes.vesselPose;

%% START EVALUATION PROCESS
startFnc() % until storage is full or rateMonitor.stop() is called

%% ROS PUBLISH CALLBACK TO EVALUATE RATE
function [] = ros_sub(~,~)

    global rateMonitor
    rateMonitor.counter = rateMonitor.counter +1;
    if rateMonitor.counter>= 2
        time = toc(rateMonitor.rateTimer);
        freq = rateMonitor.counter/time;
        disp(join(['Topic ',rateMonitor.sub.TopicName,' frequency: ',string(freq)]));
        
        rateMonitor.counter = 0;
        rateMonitor.rateTimer = tic;
        
        % save the frequency in an array for evaluation
        if rateMonitor.logIndex <= length(rateMonitor.log)
            rateMonitor.log(:,rateMonitor.logIndex) = [toc(rateMonitor.totalTime);freq]; 
            rateMonitor.logIndex = rateMonitor.logIndex+1;
        else
            disp('[warning] [topicRateMonitor.m] rateMonitor.log is full')
        end
    end
end

function [] = startFnc()
global rateMonitor
settings = delfiaSettings;
rateMonitor.counter = 0;
rateMonitor.rateTimer = tic;
rateMonitor.totalTime = tic;
rateMonitor.log = zeros(2,100000);
rateMonitor.logIndex = 1;
rateMonitor.node = ros.Node('rateMonitorNode',settings.hostname,'NodeHost',settings.myIP);
rateMonitor.sub = ros.Subscriber(rateMonitor.node,rateMonitor.topicname,rateMonitor.topictype,@ros_sub);
end

function [] = stopFnc()
global rateMonitor
delete(rateMonitor.sub)
delete(rateMonitor.node)

end

function [] = plotLog()
global rateMonitor

rateMonitor.fig = figure;

t = rateMonitor.log(1,1:rateMonitor.logIndex-1);

f = rateMonitor.log(2,1:rateMonitor.logIndex-1);
plot(t,f);
end