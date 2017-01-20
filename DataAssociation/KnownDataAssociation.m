classdef KnownDataAssociation < handle
    
    properties
    end
    
    methods
        function obj = KnownDataAssociation()
        end
        
        function DA = doAssociation(obj, b,obsDim,z,idfs,mappedIds, kfIds)
            
            z_mapped = []; % observation for known features
            idmfs = []; % Ids of features observed that are already mapped
            
            z_kf = []; % Observations for features which are in KF state
            idkfs = []; % Ids of features which are in the Kalman filter state
            
            z_new = []; % Observations to new features which need to be inserted
            idnfs= []; % Ids of features to insert into the Kalman Filter state
            
            for i = 1:size(z,2)
                
                indxMapped = find(mappedIds == idfs(i)); % is it in mapped?
                indxKF = find(kfIds == idfs(i)); % is it in KF?

                if ~isempty(indxMapped) && ~isempty(indxKF) % if mapped and in kf, treat it as mapped
                    z_mapped = [z_mapped z(:,i)];
                    idmfs = [idmfs idfs(i)];
                
                elseif ~isempty(indxMapped)
                    z_mapped = [z_mapped z(:,i)];
                    idmfs = [idmfs idfs(i)];

                elseif ~isempty(indxKF)
                    z_kf = [z_kf z(:,i)];
                    idkfs = [idkfs idfs(i)];                
                    
                else % nowhere to be found, it is new
                    z_new = [z_new z(:,i)];
                    idnfs = [idnfs idfs(i)];
                end
                
            end
            
            DA.z_mapped = z_mapped;
            DA.idmfs = idmfs;
            DA.z_kf = z_kf;
            DA.idkfs =  idkfs;
            DA.z_new = z_new;
            DA.idnfs = idnfs;
        end
    end
end