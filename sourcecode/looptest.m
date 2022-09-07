clc 
clear all

fleetCell = {Delfia,Delfia};
fleetArray = [Delfia,Delfia];


disp('build array')
t = tic;
for i = 1:100
    fleetArray(i) = Delfia;
end
toc(t)

disp('build cell array')
t = tic;
for i = 1:100
    fleetCell{i} = Delfia;
end
toc(t)

fleetCell{3}.name = 'Delfia9';
fleetArray(3).name = 'Delfia9';


%%
disp('loop stringcompare array')  
t = tic;
for vessel = fleetArray
   if strcmp(vessel.name,'Delfia9')
       disp('Delfia9')
   end
end
toc(t)

disp('loop stringcompare cell')  
t = tic;
for i=1:length(fleetCell)
    vessel = fleetCell{i};
   if strcmp(vessel.name,'Delfia9')
       disp('Delfia9')
   end
end
toc(t)

%%
remove_i = 4;
disp('Array reduce length by 1') 
t = tic;
fleetArray = [fleetArray(1:remove_i-1),fleetArray(remove_i+1:length(fleetArray))];
toc(t)
length(fleetArray)

disp('Cell array reduce length by 1') 
t = tic;
fleetCell(remove_i) =[];
toc(t)
length(fleetCell)
