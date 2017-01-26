%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Planned Localization in Unknown Maps
% Copyright 2015
% Author: Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef TwoDRangeBearingModel < ObservationModelBase
    
    properties(Constant = true)
        obsDim = 2;
        obsNoiseDim = 2;
    end
    
    properties
        sigma_b = [0.025;0.25*pi/180];%[0.025;0.3*pi/180];
        eta = [0.0;0.0];
        maxRange = 20.0;
        fov = 2*pi;
        fHandle = [];
    end
    
    methods
        
        function obj = TwoDRangeBearingModel(map, landmarkIDs, landmarkPoses)
            obj@ObservationModelBase();
            obj.landmarkIDs = landmarkIDs;
            obj.landmarkPoses = landmarkPoses;
            obj.map = map;
            
            global param;
            
            obj.sigma_b = param.rbNoiseLevelScale*obj.sigma_b;
            
            if param.doPLOT 
                param.animationPlot
                obj.fHandle.obs= plot(0,0,'r'); % drawing obs                            
            end
        end
        
        function [z,idfs] = getObservation(obj, x, varargin)
            % getObservation, Get what sensor sees conditioned on varargin.
            % We need unmapped features for mapping and mapped for
            % localization, and the measurement has to come from the true
            % location of these featuers.
            % getObservation(obj, x) all visible features with noise
            % getObservation(obj, x, 'nonoise') all visible features with no noise measurements
            % Output :
            % z Observation vector
            % idfs Ids of observations
            
            % if there are no landmarks to see
            if isempty(obj.landmarkPoses) == 1
                z=[];
                idfs = [];
                return;
                %error('There are no landmarks to see');                                
            end
            
            rangeBearing = obj.computeRangeBearing(x,obj.landmarkPoses);
            
            indxs = obj.getVisibleLandmarks(rangeBearing); % index of visible landmarks
            
            idvfs = obj.landmarkIDs(indxs); % ids of visible landmarks
            
            if ~issorted(idvfs)
                error('There is an assumption that ids are sorted so that we can make relative measurements correctly')
            end
            
            ztemp = rangeBearing(:,indxs); % no noise obs to visible landmarks
            
            if isempty(ztemp) ==1
                z=[];
                idfs = [];
                return;
            end
                                                
            if nargin == 2 % noisy observations
                
                idfs = idvfs;
                v = obj.computeObservationNoise(ztemp);
                z = ztemp + v;
                
            elseif nargin > 2 && strcmp('nonoise',varargin{1}) == 1 % nonoise
                
                idfs = idvfs;
                z = ztemp;                               
                
            else
                
                error('unknown inputs')
                
            end
            
        end
        
        function H_r = getObservationJacobian(obj,x,v,varargin)
            % Compute observation model jacobian w.r.t state
            % Inputs:
            % x: robot state
            % f: feature pose
            % Outputs:
            % H: Jacobian matrix
            
            f = varargin{1};
            
            if size(f,2) > 1
                error('Only one feature is supported for jacobian computation');
            end
            
            dx = f(1) - x(1);
            dy = f(2) - x(2);
            
            r = sqrt(dx^2 + dy^2);
            
            H_r = [-dx/r -dy/r 0;dy/r^2 -dx/r^2 -1];
            
        end
        
        function M = getObservationNoiseJacobian(obj,x,v,z)
            
            numObs = length(z);
            
            M = eye(numObs*obj.obsDim,numObs*obj.obsDim);
        end
        
        function R = getObservationNoiseCovariance(obj,x,z)
            
            if ~isempty(z)
                noise_std = repmat(obj.sigma_b,1,size(z,2)) + [z(1,:)*obj.eta(1);z(2,:)*obj.eta(2)];
            
                R = zeros(obj.obsDim*size(z,2));
            
                for i = 1:size(z,2)
                    R(obj.obsDim*i-1:obj.obsDim*i,obj.obsDim*i-1:obj.obsDim*i) = diag(noise_std(:,i).^2);
                end
            else
                R = 0;
            end
        end
        
        function H_m = getObservationFeatureJacobian(obj,x,f)
            
            if size(f,2) > 1
                error('Only one feature is supported for jacobian computation');
            end
            
            dx = f(1) - x(1);
            dy = f(2) - x(2);
            
            r = sqrt(dx^2 + dy^2);
            
            H_m = [dx/r dy/r;-dy/r^2 dx/r^2];
            
        end
        
        function P_f = getObservationFeatureCovariance(obj,x,z, idfs)
                        
            numObs = length(z);
            
            [C,findx,ib] = intersect(obj.map.mappedFeatureIDs, idfs); % find the indx of the observed features
            
            P_f = [];
            
            for i = 1:length(findx)
                
                p_f_i =  obj.map.mappedFeatureErrorCovariances(findx).P;
            
                P_f = blkdiag(P_f,p_f_i);
            end
        end
        
        function innov = computeInnovation(obj,Xprd,Zg, varargin)
            
            fPoses = varargin{1};
            
            z_prd = obj.getObservationToFeature(Xprd, fPoses, 'nonoise');
            
            innov = (Zg - z_prd);
            
            innov(2,:) = pi_to_pi(innov(2,:));
            
            innov = innov(:); % make it into a column vector
        end
        
        function z = getObservationToFeature(obj, x, fPose, varargin)
            
            % Compute exact observation
            z = obj.computeRangeBearing(x,fPose);
            
            if nargin > 3
                return;
            else
                v = obj.computeObservationNoise(z);
                
                z = z + v; % add noise if extra arg not present
            end
            
        end
        
        function rangeBearing = computeRangeBearing(obj, x, fPose)
            
            % Compute exact observation
            dx= fPose(1,:) - x(1);
            dy= fPose(2,:) - x(2);
            phi= x(3);
            rangeBearing = [sqrt(dx.^2 + dy.^2);
                atan2(dy,dx) - phi];
            
        end
        
        function v = computeObservationNoise(obj,z)
            
            noise_std = repmat(obj.sigma_b,1,size(z,2)) + [z(1,:)*obj.eta(1);z(2,:)*obj.eta(2)];
            
            v = [randn(1,size(z,2)).*noise_std(1,:);randn(1,size(z,2)).*noise_std(2,:)];
        end
        
        function indx = getVisibleLandmarks(obj,z)
            % Pass in the observation, get z's which are visible
            
            % within max range and fov
            indx = find( (abs(z(1,:)) < obj.maxRange) & (abs(pi_to_pi(z(2,:))) <= obj.fov/2));
            
        end
        
        function feats = getInverseObservation(obj, x, z)
            % use robot location and measurement to calculate feature
            % location
            % f_x = x_r + r cos(theta)
            % f_y = y_r + r sin(theta)
            feats = repmat(x(1:2),1,size(z,2)) + [z(1,:).*cos(z(2,:)+x(3));z(1,:).*sin(z(2,:)+x(3))];
        end
        
        function G = getInverseObservationJacobian(obj,x,z)
            
            if size(z,2) > 1
                error('Only one feature is supported for jacobian computation');
            end
            
            r = z(1);
            phi = z(2);
            theta = x(3);
            
            G = [1 0 -r*sin(theta+phi);
                0 1 r*cos(theta+phi)];
            
        end
        
        function W = getInverseObservationNoiseJacobian(obj,x,z)
            
            if size(z,2) > 1
                error('Only one feature is supported for jacobian computation');
            end
            
            r = z(1);
            phi = z(2);
            theta = x(3);
            
            W = [cos(theta+phi) -r*sin(theta+phi);
                sin(theta+phi) r*cos(theta+phi)];
        end
        
        function draw(obj,x,z)
            
            global param;
            
            if isempty(z), p=[]; return, end
            
            len= size(z,2);
            lnes(1,:)= zeros(1,len)+ x(1);
            lnes(2,:)= zeros(1,len)+ x(2);
            lnes(3:4,:)= transformToGlobal([z(1,:).*cos(z(2,:)); z(1,:).*sin(z(2,:))], x);
            
            len= size(lnes,2)*3 - 1;
            p= zeros(2, len);
            
            p(:,1:3:end)= lnes(1:2,:);
            p(:,2:3:end)= lnes(3:4,:);
            p(:,3:3:end)= NaN;
            
            figure(param.animationPlot);
            set(obj.fHandle.obs, 'xdata', p(1,:), 'ydata', p(2,:))
            
        end
        
    end
    
end
