%% shutdownnodes

settings = delfiaSettings();

for i = 1:settings.n_platform_controllers
    delete(pfc.controllers(i))
end
