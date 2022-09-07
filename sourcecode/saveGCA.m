ax = gca;
filename = cellfun(@(s)s(1:end-4),cellstr(strjoin(['vesselplot',strjoin(string(clock),'_')],'_')),'uni',0);
filename2 = [filename{1},'.png'];
%saveas(fig,filename{1},'png')
exportgraphics(ax,filename2)