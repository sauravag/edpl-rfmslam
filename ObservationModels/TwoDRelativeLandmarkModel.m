%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Planned Localization in Unknown Maps %
% Copyright 2016                       %
% Author: Saurav Agarwal               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef TwoDRelativeLandmarkModel < handle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Relative measurement model for 2D SLAM.
    % It is not derived from ObservationModelBase because then
    % we need to implement all abstract functions.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    properties
        relObsDim = 2;
    end
    
    methods
        
        function obj = TwoDRelativeLandmarkModel()            
        end
        
        function [z_r2f,R_r2f,idfs,numZ] = getR2FWorldObservation(obj,x,rbObsModel,headingObsModel)
            % Compute independent relative robot to feature measurements
            
            % get noise free obs to visible features
            % this gives us ids of features visible
            [z,idfs] = rbObsModel.getObservation(x, 'nonoise');
            
            numZ = size(z,2);
            
            % if I cant make any measurements, I pass on this
            if numZ < 1
                z_r2f = [];
                R_r2f = [];
                idfs = [];
                error('Cant see anything')
                %return;
            end
            
%             fprintf('NumZ = %d \n', numZ)
            
            z_r2f = zeros(obj.relObsDim*numZ,1);
            R_r2f = zeros(obj.relObsDim*numZ,obj.relObsDim*numZ);
            
             % for each feature pair
            % compute independent global frame displacements
            for i = 1:numZ
                
                % get an independent heading
                yaw = headingObsModel.getObservation(x);
                
                R_yaw = headingObsModel.sigma_b^2;                       
                
                % the noise free observations to this feature
                ztemp = z(:,i);
                
                % get independent range bearing measurements to feature
                v = rbObsModel.computeObservationNoise(ztemp); % the noise
                
                ztemp = ztemp + v;
                
                R_ztemp = rbObsModel.getObservationNoiseCovariance([],ztemp); %
                
                % compute the relative local measurement r2f
                [d_l, ~, H_to] = obj.computeLocalRelativeMeasurement([0;0],ztemp);
                
                % transform the uncertainty in range bearing to rel
                % measurement                
                R_d_l = H_to*R_ztemp*H_to';
                
                % transform local to global frame
                C = yaw2rotmat(yaw)'; %DCM
                d_w = C'*d_l;
                
                % Compute the jacobian of transpose of DCM w.r.t theta
                ctheta = C(1,1);
                stheta = C(1,2);
                dCt_dtheta = [-stheta -ctheta;ctheta -stheta];
                
                % compute jacobian of local to global transformation
                J = [C' dCt_dtheta*d_l];
                
                % propagate uncertainty in heading to relative measurement
                R_d_w = J*blkdiag(R_d_l,R_yaw)*J';
                
                % add global frame measurement to vector
                z_r2f(2*i-1:2*i,1) = d_w;
                R_r2f(2*i-1:2*i, 2*i-1:2*i) = R_d_w;
            end
            
        end
        
        function [z_f2f,R_f2f,relZID,numZ] = getF2FWorldObservation(obj,x,rbObsModel,headingObsModel)
            % Compute independent relative feature to feature measurements
            
            % get noise free obs to visible features
            % this gives us ids of features visible
            [z,idfs] = rbObsModel.getObservation(x, 'nonoise');
            
            numZ = size(z,2);
            
            % if I cant make relative measurements, I pass on this
            if numZ < 2
                z_f2f = [];
                R_f2f = [];
                relZID = [];
                error('Cant see 2 features')
%                 return;
            end
            
%             fprintf('NumZ = %d \n', numZ)

            % get relative f2f ids
            relZID = nchoosek(idfs,2);
            
            z_f2f = zeros(obj.relObsDim*size(relZID,1),1);
            R_f2f = sparse(obj.relObsDim*size(relZID,1),obj.relObsDim*size(relZID,1));
            
            % for each feature pair
            % compute independent global frame displacements
            for i = 1:size(relZID,1)
                
                % get an independent heading
                yaw = headingObsModel.getObservation(x);
                
                R_yaw = headingObsModel.sigma_b^2;
                
                % get the position in z vector of the feature pair
                p_from = find(idfs == relZID(i,1));
                p_to = find(idfs == relZID(i,2));
                
                % the noise free observations to these two features
                ztemp = z(:,[p_from,p_to]);
                
                % get independent range bearing measurements to these two
                % features
                v = rbObsModel.computeObservationNoise(ztemp); % the noise
                
                ztemp = ztemp + v;
                
                R_ztemp = rbObsModel.getObservationNoiseCovariance([],ztemp); %
                
                % compute the relative local measurement
                [d_l, H_i, H_j] = obj.computeLocalRelativeMeasurement(ztemp(:,1),ztemp(:,2));
                
                % transform the uncertainty in range bearing to rel
                % measurement
                H_d = [H_i,H_j];
                
                R_d_l = H_d*R_ztemp*H_d';
                
                % transform local to global frame
                C = yaw2rotmat(yaw)'; %DCM
                d_w = C'*d_l;
                
                % Compute the jacobian of transpose of DCM w.r.t theta
                ctheta = C(1,1);
                stheta = C(1,2);
                dCt_dtheta = [-stheta -ctheta;ctheta -stheta];
                
                % compute jacobian of local to global transformation
                J = [C' dCt_dtheta*d_l];
                
                % propagate uncertainty in heading to relative measurement
                R_d_w = J*blkdiag(R_d_l,R_yaw)*J';
                
                % add global frame measurement to vector
                z_f2f(2*i-1:2*i,1) = d_w;
                R_f2f(2*i-1:2*i, 2*i-1:2*i) = R_d_w;
            end
            
        end
        
         function [z_r2f,R_r2f,idfs,numZ] = getR2FObservation(obj,x,rbObsModel)
            % Compute independent relative robot to feature measurements
            
            % get noise free obs to visible features
            % this gives us ids of features visible
            [z,idfs] = rbObsModel.getObservation(x, 'nonoise');
            
            numZ = size(z,2);
            
            % if I cant make any measurements, I pass on this
            if numZ < 1
                z_r2f = [];
                R_r2f = [];
                idfs = [];
                error('Cant see anything')
                %return;
            end
            
%             fprintf('NumZ = %d \n', numZ)
            
            z_r2f = zeros(obj.relObsDim*numZ,1);
            R_r2f = zeros(obj.relObsDim*numZ,obj.relObsDim*numZ);
            
            % for each feature compute independent local frame displacements
            for i = 1:numZ                                  
                
                % the noise free observations to this feature
                ztemp = z(:,i);
                
                % get independent range bearing measurements to feature
                v = rbObsModel.computeObservationNoise(ztemp); % the noise
                
                ztemp = ztemp + v;
                
                R_ztemp = rbObsModel.getObservationNoiseCovariance([],ztemp); %
                
                % compute the relative local measurement r2f
                [d_l, ~, H_to] = obj.computeLocalRelativeMeasurement([0;0],ztemp);
                
                % transform the uncertainty in range bearing to rel
                % measurement                
                R_d_l = H_to*R_ztemp*H_to';                                                               
                
                % add global frame measurement to vector
                z_r2f(2*i-1:2*i,1) = d_l;
                R_r2f(2*i-1:2*i, 2*i-1:2*i) = R_d_l;
            end
            
        end
        
        function [z_f2f,R_f2f,relZID,numZ] = getF2FObservation(obj,x,rbObsModel)
            % Compute independent relative feature to feature measurements
            
            % get noise free obs to visible features
            % this gives us ids of features visible
            [z,idfs] = rbObsModel.getObservation(x, 'nonoise');
            
            numZ = size(z,2);
            
            % if I cant make relative measurements, I pass on this
            if numZ < 2
                z_f2f = [];
                R_f2f = [];
                relZID = [];
                error('Cant see 2 features')
%                 return;
            end
            
            % get relative f2f ids
            relZID = nchoosek(idfs,2);
            
            z_f2f = zeros(obj.relObsDim*size(relZID,1),1);
            R_f2f = sparse(obj.relObsDim*size(relZID,1),obj.relObsDim*size(relZID,1));
            
            % for each feature pair
            % compute independent global frame displacements
            for i = 1:size(relZID,1)                              
                
                % get the position in z vector of the feature pair
                p_from = find(idfs == relZID(i,1));
                p_to = find(idfs == relZID(i,2));
                
                % the noise free observations to these two features
                ztemp = z(:,[p_from,p_to]);
                
                % get independent range bearing measurements to these two
                % features
                v = rbObsModel.computeObservationNoise(ztemp); % the noise
                
                ztemp = ztemp + v;
                
                R_ztemp = rbObsModel.getObservationNoiseCovariance([],ztemp); %
                
                % compute the relative local measurement
                [d_l, H_i, H_j] = obj.computeLocalRelativeMeasurement(ztemp(:,1),ztemp(:,2));
                
                % transform the uncertainty in range bearing to rel
                % measurement
                H_d = [H_i,H_j];
                
                R_d_l = H_d*R_ztemp*H_d';
                                             
                % add global frame measurement to vector
                z_f2f(2*i-1:2*i,1) = d_l;
                R_f2f(2*i-1:2*i, 2*i-1:2*i) = R_d_l;
            end
            
        end
        
        function [d_ij, H_i, H_j] = computeLocalRelativeMeasurement(obj,rb_i,rb_j)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Compute the relative displacement vector from feature i to
            % feature j and corresponding jacobians.
            %
            % Input:
            % rb_i: range bearing to feature i
            % rb_j: range bearing to feature j
            %
            % Output:
            % d_ij: relative measurement
            % H_i: jacobian w.r.t z_i
            % H_j: jacobian w.r.t z_j
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Extract range and bearing
            r_i = rb_i(1);
            phi_i = rb_i(2);
            
            r_j = rb_j(1);
            phi_j = rb_j(2);
            
            cj = cos(phi_j);
            ci = cos(phi_i);
            sj = sin(phi_j);
            si = sin(phi_i);
            
            % rel reading should always use noisy measurements
            d_ij = [r_j*cj-r_i*ci;...
                r_j*sj-r_i*si];
            
            % jacobian of rel measurement function w.r.t measurement j
            % i.e., w.r.t. [r_j phi_j]
            H_j = [cj, -r_j*sj;...
                sj, r_j*cj];
            
            % jacobian of rel measurement function w.r.t measurement i
            % i.e., w.r.t. [r_i phi_i]
            H_i = [-ci, r_i*si;...
                -si, -r_i*ci];
        end
    end
    
end