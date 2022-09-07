function [] = rosinfob()

try
    rosinit
catch ME
    disp('global node unable to start up. (already running?)')
    %fprintf(2,'%s\n',ME.message);
end


disp('Topics')
rostopic list
disp('Nodes')
rosnode list
end

