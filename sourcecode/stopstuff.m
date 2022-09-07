
settings = delfiaSettings();
stopvessels.node = ros.Node('stopnode',settings.hostname,'NodeHost',settings.myIP);
stopvessels.pub = ros.Publisher(stopvessels.node,settings.topicnames.vesselActuation{3},settings.messageTypes.vesselActuation);

stopvessels.msg = rosmessage(stopvessels.pub);
stopvessels.msg.Data = [0,0,0,0];
send(stopvessels.pub,stopvessels.msg)