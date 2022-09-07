% Raw ros frequency publishing test
settings = delfiaSettings();

global rawpub

try
    stop(rawpub.timer);
end
try
    delete(rawpub.timer);
end
try
    delete(rawpub.node);
end

rawpub.node = ros.Node('rawpubnode',settings.hostname,'NodeHost',settings.myIP);
rawpub.pub = ros.Publisher(rawpub.node, settings.topicnames.vesselPose{1},settings.messageTypes.vesselPose);
rawpub.index = 1;
rawpub.f_data = zeros(2,10000);
rawpub.f_i = 1;
rawpub.s_data = zeros(2,10000);
rawpub.s_i = 1;
rawpub.f_t_last = 0;
rawpub.time = tic;
rawpub.nlog = 10;
rawpub.t_end = 4;

rawpub.timer = timer(...
                'ExecutionMode', 'fixedRate', ...           
                'Period', 1/120, ...                 
                'BusyMode', 'drop',...                    
                'TimerFcn', {@timerfnc},...
                'Name', 'rawpubtimer' );
start(rawpub.timer)

function [] = timerfnc(~,~)
    global rawpub
    a = tic;
    msg = rosmessage(rawpub.pub);
    send(rawpub.pub,msg)
    if rawpub.index >= rawpub.nlog
        
        dt_pub = toc(a);
        t = toc(rawpub.time);
        rawpub.s_data(:,rawpub.s_i) = [t;dt_pub];
        rawpub.s_i = 1+rawpub.s_i;
        
        dt = t-rawpub.f_t_last;
        f = 1/ (dt/rawpub.nlog);
        rawpub.f_data(:,rawpub.f_i) = [t;f];
        rawpub.f_t_last = t;
        rawpub.f_i = 1+rawpub.f_i;
       
        rawpub.index = 0;
        if t>rawpub.t_end
            try
            stop(rawpub.timer);
            end
            disp('done');
            plotFreq();
        end
    end
    rawpub.index = 1 + rawpub.index;
end


function [] = plotFreq()
    global rawpub
    figure
    t = rawpub.f_data(1,1:rawpub.f_i-1);
    f = rawpub.f_data(2,1:rawpub.f_i-1);
    plot(t,f);
    xlabel('t [seconds]')
    ylabel('freq [hz]')
    title('frequency broadcast over time');
    
    figure
    t2 = rawpub.s_data(1,1:rawpub.s_i-1);
    dt = rawpub.s_data(2,1:rawpub.s_i-1);
    plot(t2,dt);
    xlabel('t [seconds]')
    ylabel('publishtime [seconds]')
    title('publish speed over time');
end