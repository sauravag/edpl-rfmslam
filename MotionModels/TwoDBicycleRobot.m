%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Planned Localization in Unknown Maps
% Copyright 2015
% Author: Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2D Bicycle Robot

classdef TwoDBicycleRobot < MotionModelBase
    
    properties (Constant = true)
        stDim = 3; % state dimension
        ctDim = 2;  % control vector dimension
        wDim = 5;   % Process noise (W) dimension  % For the generality we also consider the additive noise on kinematics equation (3 dimension), but it most probably will set to zero. The main noise is a 2 dimensional noise which is added to the controls.
        sigma_b_u = 0*[0.0;1.0*pi/180]; % additive  control noise std dev
        eta_u = 0*[0.1;0.1] ; % multiplicative control noise
        eta_g_v_u = 0*[0.0;0.1*pi/180] ; % WARNING, this models the effect of speed on steering
        zeroNoise = zeros(TwoDBicycleRobot.wDim,1);
        wheelBase = 4.0; % meters
        vehFigure = [0 -TwoDBicycleRobot.wheelBase -TwoDBicycleRobot.wheelBase; 0 -2 2]; % vehicle animation
        steeringRateLimit = 30*pi/180; % the max rate of steering angle change
        maxSteeringAngle = 60*pi/180; % the max steering angle
        minDistToWaypoint = 1.0; % minimum distance
    end
    
    properties
        eta_Wg = [0.05;0.05]; % this defines the fractional error in the odomentry 
        fHandle = [];
    end
    
    methods
        
        function obj = TwoDBicycleRobot()
            obj@MotionModelBase();
            
            global param;
            
            if param.doPLOT 
                param.animationPlot
                obj.fHandle.xt= patch(0,0,'b'); % vehicle true
                obj.fHandle.xv= patch(0,0,'r'); % vehicle estimate                
            end
            
            obj.dt = param.simulator.dt;
            obj.eta_Wg = param.odoNoiseLevelScale*obj.eta_Wg;
            maxV = param.robot.maxVelocity;
            maxOmega = maxV*sin(obj.maxSteeringAngle)/obj.wheelBase;
            obj.P_Wg = diag([(maxV*obj.dt*obj.eta_Wg(1))^2, (maxV*obj.dt*obj.eta_Wg(1))^2, (maxOmega*obj.dt*obj.eta_Wg(2))^2]); % additive process noise covariance
        end
        
        function x_next = evolve(obj,x,u,w) % discrete motion model equation
            
            if obj.dt == 0.0
                error('time step size cannot be 0!');
            end
            
            control_noise = w(1:obj.ctDim);
            additive_noise = w(obj.ctDim+1:obj.wDim);
            
            v = u(1) + control_noise(1);
            g = u(2) + control_noise(2);
            
            if u(1) == 0
                additive_noise = 0*additive_noise;
            end
            
            % Change in position in local Coordinate frame of robot
            dp_local_frame = [v*obj.dt*cos(g);...
                              v*obj.dt*sin(g)];
            
            % Get rotation matrix
            Rmat = yaw2rotmat(x(3));
            
            % Initialize the next state
            x_next = zeros(obj.stDim,1);
            
            % The noise in local frame gets transformed from local frame to
            % global
            x_next(1:2,1) = x(1:2,1) + Rmat*(dp_local_frame + additive_noise(1:2,1));
            
            % Update heading
            x_next(3,1) = x(3) + v*obj.dt*sin(g)/obj.wheelBase + additive_noise(3,1);
            
            % Wrap heading
            x_next(3) = pi_to_pi(x_next(3));
        end
        
        function [odoVal, odoCov, x_next] = getOdometery(obj,x,u)
            % Compute odometery and next state based on odometery
            % using discrete motion model equation.
            
            % generate odometery noise
            w = obj.generateProcessNoise(x,u);
            
            control_noise = w(1:obj.ctDim);
            additive_noise = w(obj.ctDim+1:obj.wDim);
            
            if u(1) == 0
                additive_noise = 0*additive_noise;
            end
            
            v = u(1) + control_noise(1);
            g = u(2) + control_noise(2);
            
            % OdoVal should be in local frame
            odoVal = [v*obj.dt*cos(g);...
                v*obj.dt*sin(g);...
                v*obj.dt*sin(g)/obj.wheelBase] + additive_noise;
            
            % Get rotation matrix
            Rmat = yaw2rotmat(x(3));
            
            s = sin(g); c= cos(g);
            
            vts= v*obj.dt*s; vtc= v*obj.dt*c;
            
            % Build noise Jacobian in local frame
            G = [obj.dt*c -vts 1 0 0;...
                obj.dt*s  vtc 0 1 0;...
                obj.dt*s/obj.wheelBase v*obj.dt*c/obj.wheelBase 0 0 1];
            
            % Get noise covariance
            Q = obj.getProcessNoiseCovariance(x, u);
            
            odoCov = G*Q*G';
            
            % Initialize the next state
            x_next = zeros(obj.stDim,1);
            
            % Update position
            x_next(1:2,1) =  x(1:2,1) + Rmat*odoVal(1:2,1);
            
            % Update heading
            x_next(3) = pi_to_pi(x(3) + odoVal(3));
        end
        
        function A = getStateTransitionJacobian(obj, x,u,w)
            
            s= sin(u(2)+x(3)); c= cos(u(2)+x(3));
            
            vts= u(1)*obj.dt*s; vtc= u(1)*obj.dt*c;
            
            A = [1 0 -vts;
                0 1  vtc;
                0 0 1];
        end
        
        function B = getControlJacobian(obj, x,u,w)
            
            s= sin(u(2)+x(3)); c= cos(u(2)+x(3));
            
            vts= u(1)*obj.dt*s; vtc= u(1)*obj.dt*c;
            
            B = [obj.dt*c -vts;
                obj.dt*s  vtc;
                obj.dt*sin(u(2))/obj.wheelBase u(1)*obj.dt*cos(u(2))/obj.wheelBase];
        end
        
        function G = getProcessNoiseJacobian(obj, x,u,w)
            
            s= sin(u(2)+x(3)); c= cos(u(2)+x(3));
            
            vts= u(1)*obj.dt*s; vtc= u(1)*obj.dt*c;
            
            G = [obj.dt*c -vts cos(x(3)) -sin(x(3)) 0;
                obj.dt*s  vtc sin(x(3)) cos(x(3)) 0;
                obj.dt*sin(u(2))/obj.wheelBase u(1)*obj.dt*cos(u(2))/obj.wheelBase 0 0 1];
        end
        
        function Q = getProcessNoiseCovariance(obj, x, u)
            
            P_Un = obj.controlNoiseCovariance(u);
            
            if u(1) == 0
                Q = blkdiag(P_Un, 0*obj.P_Wg);
            else
                Q = blkdiag(P_Un, obj.P_Wg);
            end
        end
        
        function w = generateProcessNoise(obj,x,u)
            
            % generate Un
            indep_part_of_Un = randn(obj.ctDim,1);
            
            P_Un = obj.controlNoiseCovariance(u);
            
            Un = indep_part_of_Un.*diag(P_Un.^(1/2));
            
            % generate Wg
            Wg = mvnrnd(zeros(obj.stDim,1),obj.P_Wg)';
            
            w = [Un;Wg];
            
        end
        
        function P_Un = controlNoiseCovariance(obj, u)
            
            u_std = obj.eta_u.*u + obj.sigma_b_u + obj.eta_g_v_u*u(1);
            
            P_Un=diag(u_std.^2);
            
        end
        
        function draw(obj,xtrue, x)
            
            xt= transformToGlobal(obj.vehFigure,xtrue);
            xv= transformToGlobal(obj.vehFigure,x(1:3));
            set(obj.fHandle.xt, 'xdata', xt(1,:), 'ydata', xt(2,:))
            set(obj.fHandle.xv, 'xdata', xv(1,:), 'ydata', xv(2,:))
            
        end
        
        function [G,iwp]= computeSteering(obj, x, wp, iwp, G)
            %function [G,iwp]= compute_steering(x, wp, iwp, minD, G, steeringRateLimit, maxSteeringAngle, dt)
            %
            % INPUTS:
            %   x - position
            %   wp - waypoints
            %   iwp - index to current waypoint
            %   G - current steering angle
            %
            % OUTPUTS:
            %   G - new current steering angle
            %   iwp - new current waypoint
            %
            
            % determine if current waypoint reached
            cwp= wp(:,iwp);
            d2= (cwp(1)-x(1))^2 + (cwp(2)-x(2))^2;
            if d2 < obj.minDistToWaypoint^2
                iwp= iwp+1; % switch to next
                if iwp > size(wp,2) % reached final waypoint, flag and return
                    iwp=0;
                    return;
                end
                cwp= wp(:,iwp); % next waypoint
            end
            
            % compute change in G to point towards current waypoint
            deltaG= pi_to_pi(atan2(cwp(2)-x(2), cwp(1)-x(1)) - x(3) - G);
            
            % limit rate
            maxDelta= obj.steeringRateLimit*obj.dt;
            if abs(deltaG) > maxDelta
                deltaG= sign(deltaG)*maxDelta;
            end
            
            % limit angle
            G= G+deltaG;
            if abs(G) > obj.maxSteeringAngle
                G= sign(G)*obj.maxSteeringAngle;
            end
        end
    end
end