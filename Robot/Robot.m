%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Planned Localization in Unknown Maps
% Copyright 2015
% Author: Saurav Agarwal
%
% Robot class implements a generic robot
% that moves and senses. User can command the
% the robot to move or sense through the simulator.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef Robot < handle
    
    properties
        state
        observationModel
        motionModel
    end
    
    methods
        function obj = Robot(state, observationModel, motionModel)
            obj.state = state;
            obj.observationModel = observationModel;
            obj.motionModel = motionModel;
        end
    end
    
    methods
        
        function obj = move(obj, u)
            % Generate motion noise and
            % apply control action to move robot.
            
            w = obj.motionModel.zeroNoise;%generateProcessNoise(obj.state,u);
            obj.state = obj.motionModel.evolve(obj.state, u, w);
            
        end
        
        function [odoVal, odoCov, x_next] = getOdometery(obj,x,u)
            % apply control action to move robot.
            % get only the odometry readings           
            [odoVal, odoCov, x_next] = obj.motionModel.getOdometery(x, u);
        end        
        
        function [z,idfs] = sense(obj, varargin)
            % Get the sensor reading based on robot state and map.
            nVarargs = length(varargin);
            if nVarargs == 1
                [z,idfs] = obj.observationModel.getObservation(obj.state, varargin{1});
            elseif nVarargs == 2
                [z,idfs] = obj.observationModel.getObservation(obj.state, varargin{1}, varargin{2});
            else
                [z,idfs] = obj.observationModel.getObservation(obj.state);
            end
            
        end
        
    end
end