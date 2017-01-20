% Generate a figure for map file
close all;

displayLandmarks = 0; % 1 to show landmarks, 0 to turn off

fname = 'Map100kF5'; % map name

% base diretory where runs live
if isunix ==1
    [~,username] = system('whoami');
    baseDirectory = ['/home/',username(1:end-1),'/Dropbox/PLUM/MatlabFigures/'];
    % Mac is unix so have to check here
    if ismac==1
        baseDirectory = '/Users/sauravagarwal/Dropbox/PLUM/MatlabFigures/';
    end
else % is windows
    baseDirectory = '<###>';
end

saveName = strcat(baseDirectory, fname); % file name to save as

load(strcat('./Environment/',fname,'.mat')); % load the map

mfht = figure;
figure(mfht)
hold on;
if displayLandmarks
    plot(lm(1,:),lm(2,:),'*g', 'MarkerSize',1);
end
plot(wp(1,:),wp(2,:),'--d','MarkerSize',7, 'linewidth',2);
hold off;
xlim([min(wp(1,:))-100,max(wp(1,:))+100])
ylim([min(wp(2,:))-100,max(wp(2,:))+100])
xlabel('\textbf{X (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
ylabel('\textbf{Y (m)}','fontsize',14,'fontweight','bold','Interpreter','latex');
grid on;
set(mfht,'Units','Inches');
pos = get(mfht,'Position');
set(mfht,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(saveName,'-depsc');