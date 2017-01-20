function grid_landmarks()
% Place landmarks in a map given robot waypoints
% TODO: robot should be driven around the trajectory

% Logic:
% 1) Compute intermediate locations between waypoints
% 2) Place box around intermediate point
% 3) Put landmarks into box for all boxes
% 4) Save file
clc;
clear variables;

minVisibleLandmarks = 9;% minimum number of landmarks to be visible at each pose

fname = 'Map100kF9.mat';

load(strcat('./Environment/',fname)); % load lm and wp

global param

param.rbNoiseLevelScale = 1; % not important here
param.doPLOT = 0;
om = TwoDRangeBearingModel([],[],[]);

range = om.maxRange; % should be range of sensor
delta = 1; % the spacing between intermediate waypoints
spacing = 4; % spacing between landmarks

rho = 0.07; % density of landmarks
shake = 0.1; % how much shake around landmarks

lm = [];%zeros(2,minVisibleLandmarks*ceil(pathLength(wp)/delta));
lmIDs = [];

counter = 1;
for currentwp = 1:size(wp,2)-1
    
    % Compute intermediate locations between waypoints
    nextwp = currentwp+1;
    
    % vector from current to next waypoint
    v_c_n = wp(:,nextwp) - wp(:,currentwp);
    
    % distance from current to next waypoint
    d_c_n =  norm(v_c_n,2);
    
    % number of steps from c to n
    steps = floor(d_c_n / delta);
    
    for s = 1:steps
        
        % position of intermediate waypoint
        iwp = wp(:,currentwp) + s*v_c_n/steps;
        xmin = iwp(1) - range;
        xmax = iwp(1) + range;
        ymin = iwp(2) - range;
        ymax = iwp(2) + range;
        
        % get what robot sees at this pose
        [~,idfs] = om.getObservation([iwp;0],'nonoise');
        
        nz = size(idfs,2);
        
        if nz >= minVisibleLandmarks
            
            continue;
        else
            lm_iwp = [];
            
            for i = 1:ceil((xmax-xmin)/spacing)
                n = ceil((ymax-ymin)/spacing);
                
                xx = (xmin + (i-1)*spacing)*ones(1,n);
                yy = [];
                
                for j = 1:n
                    yy = [yy ymin+(j-1)*spacing];
                end
                
                lm_i = [xx;yy];
                
                lm_i = lm_i + shake*randn(size(lm_i));
                
                %                 flip = rand(1,size(lm_i,2));
                
                %                 lm_i(:,flip>rho) = [];
                
                lm_iwp = [lm_iwp lm_i];
                
            end
            
            ndeficit = minVisibleLandmarks - nz;
            
            indxs = datasample(1:size(lm_iwp,2), ndeficit, 'Replace', false);
            lm = [lm lm_iwp(:,indxs)];
            lmIDs = 1:size(lm,2);
            om.landmarkIDs = lmIDs;
            om.landmarkPoses = lm;
        end
%         else            
%             
%             ndeficit = minVisibleLandmarks - nz;
%             
%             lm_iwp = [xmin + (xmax-xmin).*rand(1,ndeficit);...
%                         ymin + (ymax-ymin).*rand(1,ndeficit)];            
% 
%             lm(:,counter:counter+ndeficit-1) = lm_iwp;
%             counter = counter + ndeficit;
%             om.landmarkIDs = 1:counter-1;
%             om.landmarkPoses = lm(:,1:counter-1);
%         end
    end
    
    fprintf('Progress: %2.2f %% \n', 100*currentwp/(size(wp,2)-1))
    
end

% lm = lm(:,1:counter-1);

clear xmin xmax ymin ymax spacing i j n lm_i rho flip shake currentwp d_c_n delta iwp nextwp range s steps v_c_n xx yy
save(strcat('./Environment/',fname),'lm','wp');

end

function L = pathLength(wp)
L = 0;
for i = 1:size(wp,2)-1
    L = L + distance(wp(:,i),wp(:,i+1));
end
end

function d = distance(p,q)
d = sqrt(sum((p-q).^2));
end
