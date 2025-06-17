%% plotMesh
% plot the running status diagram
%% Syntax
%  plotRuningStatus(t, status)
%% Description
% t: is a row vector (1*m)
%
% status: is a n*1 cell, n is the number of shafts. The data in each cell
% is 3*m matrix, m is the length of t. 1st row is omega; 2nd row is speed;
% 3rd row is acceleration.

function plotRunningStatus(t, status)

% gnerate folder to save figures
hasFolder = exist('runningStatusDiagram','dir');
if hasFolder
    delete runningStatusDiagram/*.fig;
else
    mkdir('runningStatusDiagram');
end

%%

% line name
lineName = cell(length(status),1);
for iShaft = 1:1:length(status)
    lineName{iShaft} = ['Shaft ', num2str(iShaft)];
end

%%

% plot phase
figureName = 'Phase in running status';
h1 = figure('name',figureName,'Visible', 'off');

for iShaft = 1:1:length(status)
    plot(t,status{iShaft}(1,:)); hold on
end

legend(lineName{:})
set(gcf, 'unit', 'centimeters', 'position', [5 12 15 8])
% save figure
set(gcf,'Visible','off','CreateFcn','set(gcf,''Visible'',''on'')')
figureName2 = ['runningStatusDiagram/statusOfPhase', '.fig'];
savefig(h1,figureName2,'compact')

%%

% plot speed
figureName = 'Speed in running status';
h2 = figure('name',figureName,'Visible', 'off');

for iShaft = 1:1:length(status)
    plot(t,status{iShaft}(2,:)); hold on
end

legend(lineName{:})
set(gcf, 'unit', 'centimeters', 'position', [22 12 15 8])
% save figure
set(gcf,'Visible','off','CreateFcn','set(gcf,''Visible'',''on'')')
figureName2 = ['runningStatusDiagram/statusOfSpeed', '.fig'];
savefig(h2,figureName2,'compact')
    
%%

% plot accelerate
figureName = 'Acceleration in running status';
h3 = figure('name',figureName,'Visible', 'off');

for iShaft = 1:1:length(status)
    plot(t,status{iShaft}(3,:)); hold on
end

legend(lineName{:})
set(gcf, 'unit', 'centimeters', 'position', [5 1.5 15 8])
% save figure
set(gcf,'Visible','off','CreateFcn','set(gcf,''Visible'',''on'')')
figureName2 = ['runningStatusDiagram/statusOfAcceleration', '.fig'];
savefig(h3,figureName2,'compact')

close([h1 h2 h3])

end