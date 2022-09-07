clear freqTest

%% Sim variables
settings = delfiaSettings();

global freqTest

freqTest.simPeriod = 10;                 % [sec] time between simulation updates
freqTest.publishPeriod = 0.02;             % [sec] time between ROS pose updates
freqTest.stop = @stopFnc;
freqTest.start = @startFnc;
freqTest.plot = @plotFreq;

%freqTest.node = ros.Node('freqtestnode',settings.hostname,'NodeHost',settings.myIP);
%freqTest.pubtest = ros.Publisher(freqTest.node, settings.topicnames.vesselPose{1},settings.messageTypes.vesselPose);


%% Frequency logging 
freqTest.f_index = 1;
freqTest.time = tic;
freqTest.f_data = zeros(2,10000);
freqTest.f_data_index = 1;
freqTest.logn = 6; % log frequency every n
freqTest.last_t = 0;
freqTest.endTime = 10; % seconds

freqTest.sendLogIndex = 1;
freqTest.sendLog = zeros(2,10000);

% Start the sim
startFnc();

%% Callbacks and Functions

% To start the sim
function [] = startFnc()
global freqTest


settings = delfiaSettings();
freqTest.simTime = tic;
freqTest.lastUpdate = zeros(1,settings.n_vessels);
freqTest.simTimers = cell(1,settings.n_vessels);
freqTest.sendTimers = cell(1,settings.n_vessels);
freqTest.sim_nodes = cell(1,settings.n_vessels);

%% Initialize vessel objects
disp('Initializing vessel objects')
for i = 1:settings.n_vessels
    % Create vessel object
    freqTest.fleet(i) = Delfia;
    freqTest.fleet(i).name = settings.vesselnames{i}; 
    freqTest.fleet(i).x(1:3) = [(rand(1)*8.0 -4); (rand(1)*2.0 -1); (rand(1)*pi/6 - pi/12) ]; % give random starting pose. 
    
    % Initialize ROS interaction for each vessel
    freqTest.sim_nodes{i} = ros.Node(join(['simNode_',freqTest.fleet(i).name]),settings.hostname,'NodeHost',settings.myIP);
    freqTest.fleet(i).location_publisher      =       ros.Publisher(freqTest.sim_nodes{i},settings.topicnames.vesselPose{i},settings.messageTypes.vesselPose);
    %freqTest.fleet(i).location_subscriber    =       ros.Subscriber(freqTest.sim_nodes{i},settings.topicnames.vesselPose{i},settings.messageTypes.vesselPose,{@ROS_location_callback,freqTest.fleet(i)});
    %freqTest.fleet(i).actuation_subscriber    =       ros.Subscriber(freqTest.sim_nodes{i},settings.topicnames.vesselActuation{i},settings.messageTypes.vesselActuation,{@ROS_actuation_callback,freqTest.fleet(i)});
end

disp('set up simulated vessel objects')
%% Set up timer objects
disp('Preparing simulation timers')
for i = 1:settings.n_vessels
    freqTest.simTimers{i} = timer(...
                'ExecutionMode', 'fixedRate', ...           
                'Period', freqTest.simPeriod, ...                 
                'BusyMode', 'drop',...                    
                'TimerFcn', {@timer_simulate_motion_callback, i},...
                'Name', join(['Simulation timer for ',freqTest.fleet(i).name]));
            
    freqTest.sendTimers{i} = timer(...
                'ExecutionMode', 'fixedRate', ...           
                'Period', freqTest.publishPeriod, ...                 
                'BusyMode', 'drop',...                    
                'TimerFcn', {@timer_sendpose_callback, freqTest.fleet(i),freqTest.simTime},...
                'Name', join(['ROS broadcast timer for simulated ',freqTest.fleet(i).name]));            
end
disp('Set up simulation timers')
% Start the timers
for i = 1:settings.n_vessels
    start(freqTest.simTimers{i})
    start(freqTest.sendTimers{i})
end
disp('Started simulation timers')
end

% Calls when a message is published on actuation topic
function [] = ROS_actuation_callback(~,message,vessel,~)
    %disp('ROS actuation callback. Showing message:');
    %disp('---')
    vessel.ros_sub_actuation(message); % update the vessel
end

% Calls when a message is published on actuation topic
function [] = ROS_location_callback(~,message,vessel,~)
    %vessel.ros_sub_pose(message); % XX
end

% Calls due to timer (freqTest.sendTimers)
function [] = timer_sendpose_callback(~,~,vessel,timer)
    %disp(join(['sending pose ',vessel.name,' simtime = ',string(toc(timer))]));
    a = tic;
    vessel.ros_pub_pose(toc(timer)); % Publish pose of the vessel
    %disp(join(['sending toctime =  ',string(toc(a))]));

    logFreq(toc(a))
    
    
end

function [] = logFreq(dt)
    global freqTest
    t = toc(freqTest.time);
    freqTest.sendLog(:,freqTest.sendLogIndex) = [t;dt];
    freqTest.sendLogIndex =freqTest.sendLogIndex+1;
    if freqTest.f_index >= freqTest.logn 
        dt = t-freqTest.last_t;
        avPeriod = dt/freqTest.f_index;
        f = 1/avPeriod;
        freqTest.f_data(:,freqTest.f_data_index) = [t;f]; 
        
        freqTest.last_t = t; % Set last time
        
        if freqTest.f_data_index<=length(freqTest.f_data) % Set next datapoint to write
            freqTest.f_data_index = freqTest.f_data_index+1;
        else
            disp('f_data_index out of bounds');
        end
        
        %disp('freq1')
        freqTest.f_index = 1; % reset log counter
        %disp('frequency logged')
        
        if t>freqTest.endTime
            freqTest.stop();
            freqTest.plot();
        end
        
        
    else
        freqTest.f_index = freqTest.f_index +1; % add log counter and do nothing
    end
    

end

% Calls due to timer (simTimers)
function [] = timer_simulate_motion_callback(~,~,i)
    global freqTest
    %a = tic;
    %disp(join(['simstep ',freqTest.fleet(i).name]));
    %{ 
    % XX 
    vessel = freqTest.fleet(i);
    dt = toc(freqTest.simTime) - freqTest.lastUpdate(i);
    
    eta = vessel.x(1:3);
    nu = vessel.x(4:6);
    %disp(i)
    %disp(dt)
    %disp(toc(freqTest.simTime))
    
    [M,C,D,R] = Delfia_Model(vessel.x); % Author/Source: Maurits Research paper @ RAS
    
    %% Calculate model input
    Fd = -1*D*nu;
    Fc = -1*C*nu;
    Fext = [0;0;0];
    Fb = [0;0;0];
    
    FthrSum = [0;0;0];
    for j = 1:length(vessel.r_thr) % For each thruster on vessel (e.g. = 2)
        Fthr_i = R2d(vessel.thrAngle(j))*[vessel.thrForce(j);0]; % Thruster force and angle to force vector. 
        mi = cross([vessel.r_thr(j,:),0],[Fthr_i',0]);
        FthrSum = FthrSum + [ Fthr_i(1);Fthr_i(2);mi(3)];
    end
    Fres = Fd+Fc+Fext+Fb+FthrSum;
    
    %% Calculate accellerations
    nu_dot = inv(M)*Fres; % = Accelleration at this timestep

    %% Calculate speeds
    % by stepwise integration:
    nu = nu + nu_dot*dt;
    
    %% Calculate displacements
    %R
    nu_global = R*nu;
    % by stepwise integration:
    eta = eta + nu_global*dt;
    
    %% Finish up
    vessel.x = [eta;nu];        % Set the new state
    
    
    %}
    %disp(join(['simtime =  ',string(toc(freqTest.simTime)),' ',freqTest.fleet(i).name,' steptime = ',string(toc(a))]));
    freqTest.lastUpdate(i) = toc(freqTest.simTime);
end

% To safely stop the sim
function [] = stopFnc()
global freqTest
settings = delfiaSettings();

% Delete all Simulation related timers
for i = 1:length(freqTest.simTimers)
    disp(join(['deleting Sim/send Timers of :',freqTest.fleet(i).name]));
    try
        if ~isempty(freqTest.simTimers{i})
            stop(freqTest.simTimers{i})
            delete(freqTest.simTimers{i})
            freqTest.simTimers{i} =[];
        end
    catch
        disp('some simtimers experienced an error while deleting')
    end
    try
        if ~isempty(freqTest.sendTimers{i})
            stop(freqTest.sendTimers{i})
            delete(freqTest.sendTimers{i})
            freqTest.sendTimers{i} =[];
        end
    catch
        disp('some sendTimers experienced an error while deleting')
    end
end

% Delete all ros interaction
for i = 1:settings.n_vessels
    disp(join(['deleting Simulation ROSnode of :',freqTest.fleet(i).name]));
    delete(freqTest.fleet(i).actuation_subscriber);
    delete(freqTest.fleet(i).location_subscriber);
    delete(freqTest.fleet(i).location_publisher);
    delete(freqTest.sim_nodes{i});
end

while ~isempty(freqTest.sim_nodes)
    freqTest.sim_nodes(1) =[];
end
while ~isempty(freqTest.simTimers)
    freqTest.simTimers(1) =[];
end
while ~isempty(freqTest.sendTimers)
    freqTest.sendTimers(1) =[];
end
end

function [] = plotFreq()
    global freqTest
    figure
    settings = delfiaSettings();
    t = freqTest.f_data(1,1:freqTest.f_data_index-1);
    f = freqTest.f_data(2,1:freqTest.f_data_index-1)/settings.n_vessels;
    plot(t,f);
    xlabel('t [seconds]')
    ylabel('freq [hz]')
    title(join(['publish-code frequency of ',string(settings.n_vessels),' delfias @ ',string(1/freqTest.publishPeriod),'hz publishing pose']));
    %legend('publish code frequency')
    
    figure
    t2 = freqTest.sendLog(1,1:freqTest.sendLogIndex-1);
    dt = freqTest.sendLog(2,1:freqTest.sendLogIndex-1);
    plot(t2,dt);
    xlabel('t [seconds]')
    ylabel('publishtime [seconds]')
    title(join(['publish-time of ',string(settings.n_vessels),' delfias @ ',string(1/freqTest.publishPeriod),'hz publishing pose']));
    
end