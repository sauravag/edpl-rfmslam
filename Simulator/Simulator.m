%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Planned Localization in Unknown Maps
% Copyright 2015
% Author: Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulator Class

classdef Simulator
    
    properties
        
        robot
        landmarks
        dt
        
    end
    
    methods
        
        function obj = Simulator(robot)
            global param
            obj.robot = robot;
            obj.landmarks = param.world.landmarks;
            obj.dt = param.simulator.dt;
            
        end
        
        function robot = moveRobot(obj, u)
           robot = obj.robot.move(u);
        end
        
        function [odoVal, odoCov, x_next] = getOdometery(obj,x,u)
           [odoVal, odoCov, x_next] = obj.robot.getOdometery(x,u); % calc_odo calculates the odometry data from the control u
        end        
        
        function [z,idfs] = getObservation(obj, varargin)
            
            % if we dont pass varargins like this, it screws up the cell
            % structure of varargs (becomes cell in a cell)
            nVarargs = length(varargin);
            if nVarargs == 1
                [z,idfs] = obj.robot.sense(varargin{1});
            elseif nVarargs == 2
                [z,idfs] = obj.robot.sense(varargin{1}, varargin{2});
            else
                [z,idfs] = obj.robot.sense();
            end
            
        end
        
    end
end