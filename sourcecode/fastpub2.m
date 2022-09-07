

global strack starttime ptrack sendtimer
strack = 0;
ptrack = 0;

pubnode = ros.Node('/pubnode');
subnode = ros.Node('/subnode');

pub = ros.Publisher(pubnode,'freqPose','geometry_msgs/PoseStamped');
msg = rosmessage(pub);
sub = ros.Subscriber(subnode,'freqPose','geometry_msgs/PoseStamped',{@subCallback1});


starttime = tic;
%for i = 1:1000
%    send(pub,msg);
%end

sendtimer = timer(...
    'ExecutionMode', 'fixedRate', ...
    'Period', (1/2), ...
    'BusyMode', 'drop',...
    'TimerFcn', {@timerfnc,pub,msg},...
    'Name', 'rawpubtimer' );

start(sendtimer)

%{
delete(pubnode)
delete(subnode)
%}
%clear(node)

function [] = timerfnc(~,~,pub,msg)
    send(pub,msg);
    global ptrack
    ptrack = ptrack +1;
    if ptrack >= 900
        global sendtimer starttime
        stop(sendtimer)
        disp('pub 900 time:');
        disp(toc(starttime));
    end
end


function [] = subCallback1(~,~)
    global strack
    strack = strack+1;   
    
    if strack ==1
        global starttime
        disp('Firs sub callback at:');
        disp(toc(starttime));
    elseif strack ==450
        global starttime
        disp('450 subs at');
        disp(toc(starttime));
    end
end