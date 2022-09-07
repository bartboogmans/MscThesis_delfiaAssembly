v = Delfia(1,0,pi/3,'delfia4');
fig=uifigure;
ax = uiaxes(fig);
ax.NextPlot ='add';
ax.View = [90 -90];

v.plotVessel(ax);

ax.XLabel.String = 'X';
ax.YLabel.String = 'Y';

%% to continue
% Plot a delfia instead of a vessel
% Plot a MultiVesselPlatform
% 
% continue with the simulation of the vessels

