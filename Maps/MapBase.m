classdef MapBase < handle
    
    properties
        mappedFeatureIDs; % vectorization of mapped keeps things faster.
        mappedFeaturePoses;
        mappedFeatureErrorCovariances;
        mappedFeatureTotalErrorCovariances;
        unmappedFeatures; % features that the robot is current estimating but not yet fixed.
        numMappedFeatures;
        numUnmappedFeatures
    end
    
    methods(Abstract)
        
        update(obj, obsModel, robotBelief, z, ids, toMap);
        
        insertFeature(obj, featureID, featurePdf);
        
        isMapped(obj, featureID);

%         obj = deleteFeature(obj, featureID);
%         
%         obj = updateFeature(obj, f);
    end
        
end