% Class to manage map
classdef TwoDMap < MapBase
    
    properties
        pathToSaveFigs = '';
        noPlot = 0; % if 0 will plot, set to 1 to stop plotting
        mappedFeaturesCovarianceMatrix;
    end
    methods
        
        function obj = TwoDMap(varargin)
            obj.mappedFeatureIDs = [];
            obj.mappedFeaturePoses = [];
            obj.mappedFeaturesCovarianceMatrix = [];
%             obj.mappedFeatureTotalErrorCovariance = [];
            obj.unmappedFeatures = struct();
            obj.numMappedFeatures = 0;
            obj.numUnmappedFeatures = 0;
            
            if length(varargin) > 0
                obj.pathToSaveFigs = varargin{1};
            end
        end
        
        function obj = update(obj, obsModel, robotBelief, z, idfs, toMap)
            % For features in the filter do a freuentist update,
            % can be only applied when robot is stopped.
            global param;
            
            for nfeat = 1:size(z,2)
                
                featureMeasurement = z(:,nfeat);
                featureID = idfs(nfeat);
                
                robotStateDim = 3;
                x  =  robotBelief.mean(1:robotStateDim);
                P  =  robotBelief.covariance;
                
                unmappedFeatureIndex = obj.isBeingEstimated(featureID);
                
                % unseen feature but we dont want to map
                if isempty(unmappedFeatureIndex) == 1 && toMap == 0
                    break;
                end
                
                if isempty(unmappedFeatureIndex) == 1  % If feature was not previously seen
                    
                    obj.numUnmappedFeatures = obj.numUnmappedFeatures + 1;% count up as you have seen a new feature
                    unmappedFeatureIndex = obj.numUnmappedFeatures;
                    
                    obj.unmappedFeatures(unmappedFeatureIndex).id = featureID;
                    obj.unmappedFeatures(unmappedFeatureIndex).numObservations = 1; % initialize count how many times we see
                    
                    obj.unmappedFeatures(unmappedFeatureIndex).fpdf = [];
                    obj.unmappedFeatures(unmappedFeatureIndex).fpdf.mean = [];
                    obj.unmappedFeatures(unmappedFeatureIndex).fpdf.covariance = [];
                    obj.unmappedFeatures(unmappedFeatureIndex).estimateHist = [];
                    obj.unmappedFeatures(unmappedFeatureIndex).data = [];
                    
                    %                     if obj.noPlot == 0
                    %                         obj.unmappedFeatures(unmappedFeatureIndex).plotHandle = figure;
                    %                         set(obj.unmappedFeatures(unmappedFeatureIndex).plotHandle,'name',['Feature ID #',num2str(featureID)],'WindowStyle', 'docked');
                    %                     end
                else % if feature was previously seen
                    % count up how many times we have seen this feature
                    obj.unmappedFeatures(unmappedFeatureIndex).numObservations = obj.unmappedFeatures(unmappedFeatureIndex).numObservations +1;
                end
                
                obj.unmappedFeatures(unmappedFeatureIndex).data = [obj.unmappedFeatures(unmappedFeatureIndex).data featureMeasurement];
                
                if obj.unmappedFeatures(unmappedFeatureIndex).numObservations >= param.maxObservations
                    
                    avgRange = mean(obj.unmappedFeatures(unmappedFeatureIndex).data(1,:));
                    avgBearing = mean(obj.unmappedFeatures(unmappedFeatureIndex).data(2,:));
                    
                    obj.unmappedFeatures(unmappedFeatureIndex).fpdf.mean =  obsModel.getInverseObservation(x,[avgRange;avgBearing]);
                    
                    robStateDim = 3;
                    
                    % Compute the necessary jacobians
                    Gr =  obsModel.getInverseObservationJacobian(x(1:robStateDim),featureMeasurement);
                    W = obsModel.getInverseObservationNoiseJacobian(x(1:robStateDim),featureMeasurement);

                    R = obsModel.getObservationNoiseCovariance(x(1:robStateDim),featureMeasurement) / obj.unmappedFeatures(unmappedFeatureIndex).numObservations;

                    % compute covariance matrix
                    obj.unmappedFeatures(unmappedFeatureIndex).fpdf.covariance= Gr*P(1:robStateDim,1:robStateDim)*Gr' + W*R*W'; % feature cov
                    
                    obj.insertFeature(unmappedFeatureIndex);
                end
            end
            
        end
        
        function Indx = isBeingEstimated(obj, featureID)
            % Returns index of feature if found in unmapped features struct
            % i.e. checking if we had already started estimating this
            % feature or not.
            Indx = structfind(obj.unmappedFeatures, 'id', featureID);
        end        
        
        function obj = insertFeatureMC(obj, featureID, pose, covariance)
            
            global param;
            
            obj.mappedFeatureIDs(obj.numMappedFeatures+1)  = featureID;
            obj.mappedFeaturePoses(:,obj.numMappedFeatures+1)  = pose;
            obj.mappedFeatureErrorCovariances(obj.numMappedFeatures+1).P = covariance;
            
            obj.numMappedFeatures = obj.numMappedFeatures + 1;
            
            unmappedFeatureIndex = obj.isBeingEstimated(featureID);
            
            % remove from unmapped once it is mapped
            obj.unmappedFeatures(unmappedFeatureIndex) = [];
            obj.numUnmappedFeatures = obj.numUnmappedFeatures - 1;
        end
                
        function obj = insertFeature(obj, unmappedFeatureIndex)
            
            global param;
            
            obj.mappedFeatureIDs(obj.numMappedFeatures+1)  = obj.unmappedFeatures(unmappedFeatureIndex).id;
            obj.mappedFeaturePoses(:,obj.numMappedFeatures+1)  = obj.unmappedFeatures(unmappedFeatureIndex).fpdf.mean;
            obj.mappedFeatureErrorCovariances(obj.numMappedFeatures+1).P  = obj.unmappedFeatures(unmappedFeatureIndex).fpdf.covariance;
            
            obj.numMappedFeatures = obj.numMappedFeatures + 1;
            
%             if obj.noPlot == 0
%                 obj.draw(); % only update when you added something new
%                 obj.drawFeatureBelief(unmappedFeatureIndex); % draw feature belief
%                 saveas(obj.unmappedFeatures(unmappedFeatureIndex).plotHandle,[obj.pathToSaveFigs,'feat',num2str(obj.unmappedFeatures(unmappedFeatureIndex).id),'.jpg'])
%                 close(obj.unmappedFeatures(unmappedFeatureIndex).plotHandle);
%             end
            
            % remove from unmapped once it is mapped
            obj.unmappedFeatures(unmappedFeatureIndex) = [];
            obj.numUnmappedFeatures = obj.numUnmappedFeatures - 1;
            
        end
        
        function Sigma = extractMappedFeaturesCovariance(obj,idmfs)
            
             [idpfs, indx, ib] = intersect(obj.mappedFeatureIDs, idmfs);
             
             fsize = 2;            
             nr = fsize*size(indx,2);
             nc = size(obj.mappedFeaturesCovarianceMatrix,2);
             
             E = zeros(nr,nc);
             
             for i=1:length(indx)
                 
                 % the index of feature in map
                 j = indx(i);                 
                 E(2*i-1:2*i,2*j-1:2*j) = eye(fsize);                   
             end
             
             Sigma = E*obj.mappedFeaturesCovarianceMatrix*E';
             
        end
        
        function YesNo = isMapped(obj, featureID)
            YesNo = -1;
        end
        
    end
end

