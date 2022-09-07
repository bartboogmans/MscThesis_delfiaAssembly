global t

settings = delfiaSettings();
t.node = ros.Node('rpsTestNode1',settings.hostname,'NodeHost',settings.myIP);
t.i = 1;
t.pub = ros.Publisher(t.node,settings.topicnames.vesselActuation{t.i},settings.messageTypes.vesselActuation);

t.stoptimer = timer('ExecutionMode','singleShot','StartDelay',10,'Period',5.0,'TimerFcn',{@timerFCN1});

t.msg = rosmessage(t.pub);
t.msg.Data = [0,20,0,pi];
send(t.pub,t.msg);
t.ticstamp = tic;

start(t.stoptimer)

function timerFCN1(~,~)
global t
t.msg.Data = [0,0,0,pi];
send(t.pub,t.msg);
disp(toc(t.ticstamp));

stop(t.stoptimer);
delete(t.stoptimer);
delete(t.node);
delete(t.pub);



end