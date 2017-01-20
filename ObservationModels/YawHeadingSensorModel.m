classdef  YawHeadingSensorModel < ObservationModelBase
    % Unbiased noisy heading sensor for 2D robots
    
    properties(Constant = true)
        obsDim = 1; % dimension of the observation vector
        obsNoiseDim = 1; % observation noise dimension. In some other observation models the noise dimension may be different from the observation dimension.
    end
    
     properties
        sigma_b = 0.005*pi/180; %(18 arcseconds) the noise std dev, 1 degree = 3600 arc seconds
    end
        
    methods
        
        function obj = YawHeadingSensorModel()
            obj@ObservationModelBase();
        end
        
        function z = getObservation(obj,x, varargin)
            % feed the true state 'x' 
            trueYaw = x(3);
            z = trueYaw + randn*obj.sigma_b;
        end
                
        function H_r = getObservationJacobian(obj,x,v,varargin)
            H_r= zeros(1,length(x));
            H_r(3)= 1;
        end
        
        function M = getObservationNoiseJacobian(obj,x,v,z)
            M = 1;
        end
        
        function R = getObservationNoiseCovariance(obj,x,z)
            R = obj.sigma_b^2;
        end
        
        function H_m = getObservationFeatureJacobian(obj,x,f)
            error('should not be called');
        end
        
        function P_f = getObservationFeatureCovariance(obj,x,z)
            error('should not be called');
        end
        
        function innov = computeInnovation(obj,Xprd,Zg,varargin)
            innov = pi_to_pi(Zg - Xprd(3));
        end
        
        function feats = getInverseObservation(obj, x, z)
            error('should not be called');
        end
        
        function G = getInverseObservationJacobian(obj,x,z)
            error('should not be called');
        end
            
        function W = getInverseObservationNoiseJacobian(obj,x,z)
            error('should not be called');
        end
    end
    
end