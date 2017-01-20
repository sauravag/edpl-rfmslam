%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Planned Localization in Unknown Maps %
% Copyright 2016                       %
% Author: Saurav Agarwal               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef RFMSLAM2D < handle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 2D Relative Measurements-based Orientation
    % and Landark Estimation
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
        adjMatrix = []; % graph adjacency matrix
        nodes = struct('xtrue',[],'idvfs',[],'z',[],'R_z',[],'DCM',[],'trueDCM',[],'relZID',[],'d_l',[],'R_d_l',[],'r2f_l',[],'R_r2f_l',[]); % struct containing all information at a node
        edges = []; % an array containing edges [from to]
        relativeRotConstraint = struct('relRotMat',[],'kappa',[],'deltaTheta',[],'sigmaDeltaTheta',[]); % struct containing all relative rotation constraints
        proprioceptiveOdo = struct('mean',[],'cov',[]); % struct containing proprioceptive odometery data
        numNodes = 0; % number of nodes in graph
        numEdges = 0; % number of edges in graph
        totalRelativeMeasurements = 0;% total number of relative measurements
        totalR2FMeasurements = 0;% total number of relative measurements from robot to feature
        uniqueLandmarkIDs = []; % vector of unique landmark IDs
        estimatedPose = []; % The vector of estimated robot position
        poseCovariance =[]; % The vector of estimated robot position covariance
        estimatedFeatures = []; % The array of estimated feature poses
        featCovariance = []; % The covariance matrix of estimated feature poses
        tolerance = 1e-12; % The tolerance for min eigen value or det we require for PD matrix
        fHandle = [];
        initialGuessTraj = []; % the odometery based initial guess of trajectory, GTSAM uses this
        initialSigma = []; % anchor pose uncertainty
        gotPreComputedEdges = 0; % set to 1 if edges provided
    end
    
    methods
        function obj = RFMSLAM2D(initialSigma, varargin)
            %             obj.fHandle.xguess = plot(0,0,'b'); % proprioceptive odometery based initial guess of trajectory
            %             obj.fHandle.xr = plot(0,0,'c'); % estimated trajectory
            %             obj.fHandle.xf = plot(0,0,'+c'); % estimated features
            %             obj.fHandle.cov= plot(0,0,'c'); % covariance ellipses
            obj.initialSigma = initialSigma;
            
            nVarargs = length(varargin);
            
            % if 1 vararg present, we load data from graph file
            if nVarargs == 1
                obj.loadGraphFromFile(varargin{1});
            end
            
            %if 2 vararg present, we load data from graph file and edges
            if nVarargs == 2
                obj.loadGraphFromFile(varargin{1});
                obj.gotPreComputedEdges = 1;
                obj.edges = varargin{2};
                obj.numEdges = size(obj.edges,1);
            end
        end
        
        function insertNode(obj, xtrue, idvfs, z, R_z, propOdoVal, propOdoCov)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Insert node into graph
            %
            % Input:
            % z: observations at node
            % idvfs: ids of visible features
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % increment number of nodes in graph
            obj.numNodes = obj.numNodes + 1;
            
            % store the data in each node
            obj.nodes(obj.numNodes).xtrue = xtrue; % the true pose
            obj.nodes(obj.numNodes).idvfs = idvfs; % ids of visible feats
            obj.nodes(obj.numNodes).z = z; % the measurments
            obj.nodes(obj.numNodes).R_z = R_z; % measurement noise covariance
            obj.nodes(obj.numNodes).DCM = zeros(2,2); % initialize an unknown DCM
            obj.nodes(obj.numNodes).trueDCM = yaw2rotmat(xtrue(3))'; % get the true DCM
            obj.totalR2FMeasurements = obj.totalR2FMeasurements + size(z,2); % increment the total count of robot to feature position measurements
            
            % First node is anchor node (known pose)
            if obj.numNodes == 1
                obj.nodes(obj.numNodes).DCM = yaw2rotmat(xtrue(3,1))'; % transpose of rot mat
            end
            
            % There is no odo to first node as it is anchor
            if obj.numNodes > 1
                obj.insertPropOdoEdge(propOdoVal, propOdoCov, obj.numNodes-1); % odometery is from the previous to this node
            end
        end
        
        function insertPropOdoEdge(obj,propOdoVal, propOdoCov, fromNode)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Insert a proprioceptive odometery edge between successive
            % nodes.
            %
            % Input:
            % propOdoVal: The estimated odometer value (mean)
            % propOdoCov: Error covariance in odometery reading
            %             fromNode: The node from which this odometery
            %             is estimated, the toNode = fromNode + 1
            %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % The number of proprioceptive odometery edges is 1 less than
            % the number of poses, we can use the fromNode id as the index
            % of this edge.
            obj.proprioceptiveOdo(fromNode).mean = propOdoVal;
            obj.proprioceptiveOdo(fromNode).cov = propOdoCov;
        end
        
        function results = solve(obj)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Solve the full problem
            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            fprintf('Start solving RFMSLAM...\n')
            
            tstartRFMSolve = tic;
            
            % Process local measurements, i.e., Robot2Feature,
            % Feature2Feature
            obj.processLocalMeasurements();
            
            % First build the graph edges
            if obj.gotPreComputedEdges == 0
                obj.buildGraphEdges();
            else % or use precomputed information
                obj.usePreComputedGraphEdges();
            end
            
            % Solve for heading, if heading not available
            [theta_full, Omega_theta, tOrientationOpt] = obj.solveOrientation();
            
            % Solve for robot trajectory & feature poses and estimate covariance
            timeToSolve = obj.solveRobotFeaturePositions(theta_full, Omega_theta);
            
            tStopRFMSolve = toc(tstartRFMSolve);
            
            results.estimatedPose  = obj.estimatedPose;
            results.estimatedFeats = obj.estimatedFeatures;
            results.poseCovariance = obj.poseCovariance;
            results.featCovariance = obj.featCovariance;
            results.tOrientationOpt = tOrientationOpt;
            results.totalTime = tOrientationOpt + timeToSolve;
            
            if obj.gotPreComputedEdges == 0
                results.edges = obj.edges;
            end
            
            fprintf('Done, time to do solve function = %f s \n',tStopRFMSolve)
            
        end
        
        function processLocalMeasurements(obj)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Solve estimation problem on graph
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            fprintf('Building local measurements...\n')
            
            relObsDim = 2; % dimension of 1 relative measurement
            
            % Get the relative observation model
            relObsModel = TwoDRelativeLandmarkModel();
            
            %% Step 1
            % Compute all the local realtive r2f measurements at each node.
            % Basically compute the position of feature in local frame and
            % error covariance in this position estimate
            for i = 1:1:obj.numNodes
                
                % get the data (landmarks ids, range bearing z)
                idvfs = obj.nodes(i).idvfs;
                
                if size(idvfs,2) == 0 % saw nothing
                    continue;
                end
                
                z = obj.nodes(i).z;
                numZ = size(z,2);
                
                % Initially just stack up all the idvfs
                obj.uniqueLandmarkIDs = [obj.uniqueLandmarkIDs,idvfs];
                
                % local relative measurement vector
                r2f_l = zeros(relObsDim*numZ,1);
                
                % local relative measurement jacobian
                H_r2f_l = zeros(relObsDim*numZ,relObsDim*numZ);
                
                for j = 1:numZ
                    
                    % get range-bearing for the feature
                    rb_toFeat = z(:,j);
                    
                    % get the local relative measurement and jacobians
                    [r2f_j_l, H_r2f_j_l_from, H_r2f_j_l_to]  = relObsModel.computeLocalRelativeMeasurement([0;0],rb_toFeat);
                    
                    % insert value
                    r2f_l(relObsDim*j-1:relObsDim*j,1) = r2f_j_l;
                    
                    % insert jacobians in correct place
                    H_r2f_l(relObsDim*j-1:relObsDim*j,relObsDim*j-1:relObsDim*j) = H_r2f_j_l_to;
                end
                
                % get the range bearing measurement covariance
                R_z = obj.nodes(i).R_z;
                
                % transform R_z to relative measurement space
                R_r2f_l = H_r2f_l*R_z*H_r2f_l';
                
                % Enfore Symmetric PD
                R_r2f_l = nearPD((R_r2f_l + R_r2f_l')/2);
                
                % store values in node
                obj.nodes(i).r2f_l = r2f_l;
                obj.nodes(i).R_r2f_l = R_r2f_l;
                
            end
            
            % Extract just the unique IDs
            obj.uniqueLandmarkIDs = unique(obj.uniqueLandmarkIDs);
            
            %% Step 2
            % Compute all the local realtive f2f measurements at each node.
            % We will need to compute the ids for the pairwise
            % measurements, the mean value and estimated covariance.
            for i = 1:1:obj.numNodes
                
                % get the data (landmarks ids, range bearing z)
                idvfs = obj.nodes(i).idvfs;
                
                if size(idvfs,2) < 2 % saw less than 2 features, cant do relative measurement
                    %                     warning('No relative measurement');
                    obj.nodes(i).relZID = [];
                    continue;
                end
                
                z = obj.nodes(i).z;
                numZ = size(z,2);
                
                % Compute relative landmark pairs
                relZID = nchoosek(idvfs,2);
                
                % the number of relative measurements
                numRelObs = size(relZID,1);
                
                % increment the total count
                obj.totalRelativeMeasurements = obj.totalRelativeMeasurements + numRelObs;
                
                % local relative measurement vector
                d_l = zeros(relObsDim*numRelObs,1);
                
                % local relative measurement jacobian
                H_d_l = zeros(relObsDim*numRelObs,relObsDim*numZ);
                
                for j = 1:numRelObs
                    
                    % relative z goes: fromFeat---->toFeat
                    indx_fromFeat = find(idvfs == relZID(j,1));
                    indx_toFeat = find(idvfs == relZID(j,2));
                    
                    % get range-bearing for these two features
                    rb_fromFeat = z(:,indx_fromFeat);
                    rb_toFeat = z(:,indx_toFeat);
                    
                    % get the local relative measurement and jacobians
                    [d_j_l, H_d_j_l_from, H_d_j_l_to]  = relObsModel.computeLocalRelativeMeasurement(rb_fromFeat,rb_toFeat);
                    
                    % insert value
                    d_l(relObsDim*j-1:relObsDim*j,1) = d_j_l;
                    
                    % insert jacobians in correct place
                    H_d_l(relObsDim*j-1:relObsDim*j,relObsDim*indx_fromFeat-1:relObsDim*indx_fromFeat) = H_d_j_l_from;
                    H_d_l(relObsDim*j-1:relObsDim*j,relObsDim*indx_toFeat-1:relObsDim*indx_toFeat) = H_d_j_l_to;
                end
                
                % get the range bearing measurement covariance
                R_z = obj.nodes(i).R_z;
                
                % transform R_z to relative measurement space
                R_d_l = H_d_l*R_z*H_d_l';
                
                % Enfore Symmetric PD
                R_d_l = nearPD((R_d_l + R_d_l')/2);
                
                % store values in node
                obj.nodes(i).relZID = relZID;
                obj.nodes(i).d_l = d_l;
                obj.nodes(i).R_d_l = R_d_l;
                
            end
            
            fprintf('Done building local measurements, Total Number of Relative Measurements: %u \n', obj.totalRelativeMeasurements)
            
        end
        
        function buildGraphEdges(obj)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Solve estimation problem on graph
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            fprintf('Building Graph...\n')
            tStart = tic;
            
            % Initialize the edges, min num of edges is the number of
            % nodes - 1.
            obj.edges = zeros(obj.numNodes-1,2);
            
            %% Step 1
            % Build the graph edges for every pair of successive nodes.
            % Obviously successive nodes have proprioceptive odometery edges,
            % but may not have common relative measurements (very rare
            % though)
            for fromNode = 1:obj.numNodes-1 % odometery is upto last pose
                
                % toNode is just the next node
                toNode = fromNode + 1;
                
                % increment number of edges
                obj.numEdges = obj.numEdges + 1;
                
                % add the edge data into array
                obj.edges(obj.numEdges,:) = [fromNode toNode];
                
                % Check if there are any common relative measurements
                relZID_from = obj.nodes(fromNode).relZID;
                relZID_to = obj.nodes(toNode).relZID;
                
                % Check if either pose has 0 relative measurements
                if isempty(relZID_from) || isempty(relZID_to)
                    % If yes, then keep these variables empty
                    commonRelZID = [];
                    icFrom = [];
                    icTo = [];
                    %                     warning('No relative measurements here')
                else
                    % If both not empty, find the intersection
                    [commonRelZID, icFrom, icTo] = intersect(relZID_from,relZID_to,'rows');
                end
                
                [relRotMat, kappa, deltaTheta, sigmaDeltaTheta] = obj.computeRelativeOrientationConstraint(fromNode, toNode, commonRelZID, icFrom, icTo);
                
                if kappa < 0
                    error('Kappa should not be negative with proprioceptive odometery available!');
                end
                
                obj.relativeRotConstraint(obj.numEdges).relRotMat = relRotMat;
                obj.relativeRotConstraint(obj.numEdges).kappa = kappa;
                obj.relativeRotConstraint(obj.numEdges).deltaTheta = deltaTheta;
                obj.relativeRotConstraint(obj.numEdges).sigmaDeltaTheta = sigmaDeltaTheta;
                
                % Chain the odometery to get the initial guess
                obj.nodes(toNode).DCM = relRotMat'*obj.nodes(fromNode).DCM;
            end
            
            %% Step 2
            % Build the graph loop closure edges. We need to check every node with
            % every other node, ignoring successive nodes.
            possibleEdges = nchoosek(1:obj.numNodes,2);
            for pe = 1:size(possibleEdges,1)
                
                fromNode = possibleEdges(pe,1);
                toNode = possibleEdges(pe,2);
                
                % nothing to do for the same or successive node
                if abs(fromNode - toNode) < 2 %fromNode == toNode || toNode == fromNode + 1
                    continue;
                end
                
                commonRelZID = [];
                icFrom = [];
                icTo = [];
                
                % Check if there are any common relative measurements
                relZID_from = obj.nodes(fromNode).relZID;
                
                if isempty(relZID_from)
                    continue;
                end
                
                relZID_to = obj.nodes(toNode).relZID;
                
                if isempty(relZID_to)
                    continue;
                end
                
                % If both not empty, find the intersection
                [commonRelZID, icFrom, icTo] = intersect(relZID_from,relZID_to,'rows');
                
                if isempty(commonRelZID) % no relative feat constraints
                    continue; % you can skip any further calcs
                else
                    % if there are common measurements
                    [relRotMat, kappa, deltaTheta, sigmaDeltaTheta] = obj.computeRelativeOrientationConstraint(fromNode, toNode, commonRelZID, icFrom, icTo);
                    
                    % increment edge count and add edge accordingly
                    obj.numEdges = obj.numEdges + 1;
                    obj.edges(obj.numEdges,:) = [fromNode toNode];
                    obj.relativeRotConstraint(obj.numEdges).relRotMat = relRotMat;
                    obj.relativeRotConstraint(obj.numEdges).kappa = kappa;
                    obj.relativeRotConstraint(obj.numEdges).deltaTheta = deltaTheta;
                    obj.relativeRotConstraint(obj.numEdges).sigmaDeltaTheta = sigmaDeltaTheta;
                end
            end
            
            % Quickly solve first guess of orientation using eigen value
            % method
            obj.solveInitialOrientation();
            
            %% Step 3
            % Redo the orientation constraints
            for edgeNum = 1:size(obj.edges,1)
                
                fromNode = obj.edges(edgeNum,1);
                toNode = obj.edges(edgeNum,2);
                
                relZID_from = obj.nodes(fromNode).relZID;
                relZID_to = obj.nodes(toNode).relZID;
                
                if isempty(relZID_from) || isempty(relZID_to)
                    % If yes, then keep these variables empty
                    commonRelZID = [];
                    icFrom = [];
                    icTo = [];
                    continue;
                end
                
                [commonRelZID, icFrom, icTo] = intersect(relZID_from,relZID_to,'rows');
                [relRotMat, kappa, deltaTheta, sigmaDeltaTheta] = obj.computeRelativeOrientationConstraint(fromNode, toNode, commonRelZID, icFrom, icTo);
                
                if kappa > 0
                    % increment edge count and add edge accordingly
                    obj.relativeRotConstraint(edgeNum).relRotMat = relRotMat;
                    obj.relativeRotConstraint(edgeNum).kappa = kappa;
                    obj.relativeRotConstraint(edgeNum).deltaTheta = deltaTheta;
                    obj.relativeRotConstraint(edgeNum).sigmaDeltaTheta = sigmaDeltaTheta;
                end
            end
            
            tCompute = toc(tStart);
            
            fprintf('Done! Number of Edges: %u, Graph Construction Time: %f s \n',obj.numEdges, tCompute);
            
        end
        
        function usePreComputedGraphEdges(obj)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Solve estimation problem on graph
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            fprintf('Building relative orientation constraints using pre-computed graph...\n')
            tStart = tic;
            
            %% Step 1
            % do the graph edges for every pair of successive nodes.
            % Obviously successive nodes have proprioceptive odometery edges,
            % but may not have common relative measurements (very rare
            % though)
            for edgeNum = 1:obj.numNodes-1 % the number of odometery edges is 1 less than num nodes
                
                fromNode = obj.edges(edgeNum,1);
                toNode = obj.edges(edgeNum,2);
                
                if toNode ~= fromNode + 1
                    error('I expect that the first numNodes-1 edges are odometeric constraints');
                end
                
                % Check if there are any common relative measurements
                relZID_from = obj.nodes(fromNode).relZID;
                relZID_to = obj.nodes(toNode).relZID;
                
                % Check if either pose has 0 relative measurements
                if isempty(relZID_from) || isempty(relZID_to)
                    % If yes, then keep these variables empty
                    commonRelZID = [];
                    icFrom = [];
                    icTo = [];
                    %                     warning('No relative measurements here')
                else
                    % If both not empty, find the intersection
                    [commonRelZID, icFrom, icTo] = intersect(relZID_from,relZID_to,'rows');
                end
                
                [relRotMat, kappa, deltaTheta, sigmaDeltaTheta] = obj.computeRelativeOrientationConstraint(fromNode, toNode, commonRelZID, icFrom, icTo);
                
                if kappa < 0
                    error('Kappa should not be negative with proprioceptive odometery available!');
                end
                
                obj.relativeRotConstraint(edgeNum).relRotMat = relRotMat;
                obj.relativeRotConstraint(edgeNum).kappa = kappa;
                obj.relativeRotConstraint(edgeNum).deltaTheta = deltaTheta;
                obj.relativeRotConstraint(edgeNum).sigmaDeltaTheta = sigmaDeltaTheta;
                
                % Chain the odometery to get the initial guess
                obj.nodes(toNode).DCM = relRotMat'*obj.nodes(fromNode).DCM;
            end
            
            %% Step 2
            % do the orientation constraints for non successive
            for edgeNum = obj.numNodes:size(obj.edges,1)
                
                fromNode = obj.edges(edgeNum,1);
                toNode = obj.edges(edgeNum,2);
                
                relZID_from = obj.nodes(fromNode).relZID;
                relZID_to = obj.nodes(toNode).relZID;
                
                if isempty(relZID_from) || isempty(relZID_to)
                    % If yes, then keep these variables empty
                    commonRelZID = [];
                    icFrom = [];
                    icTo = [];
                    continue;
                end
                
                [commonRelZID, icFrom, icTo] = intersect(relZID_from,relZID_to,'rows');
                [relRotMat, kappa, deltaTheta, sigmaDeltaTheta] = obj.computeRelativeOrientationConstraint(fromNode, toNode, commonRelZID, icFrom, icTo);
                
                if kappa > 0
                    % increment edge count and add edge accordingly
                    obj.relativeRotConstraint(edgeNum).relRotMat = relRotMat;
                    obj.relativeRotConstraint(edgeNum).kappa = kappa;
                    obj.relativeRotConstraint(edgeNum).deltaTheta = deltaTheta;
                    obj.relativeRotConstraint(edgeNum).sigmaDeltaTheta = sigmaDeltaTheta;
                end
            end
            
            % Quickly solve first guess of orientation using eigen value
            % method
            flagig = obj.solveInitialOrientation();
            
            %% Step 3
            if flagig ~= -1
                % redo the orientation constraints
                for edgeNum = 1:size(obj.edges,1)
                    
                    fromNode = obj.edges(edgeNum,1);
                    toNode = obj.edges(edgeNum,2);
                    
                    relZID_from = obj.nodes(fromNode).relZID;
                    relZID_to = obj.nodes(toNode).relZID;
                    
                    if isempty(relZID_from) || isempty(relZID_to)
                        % If yes, then keep these variables empty
                        commonRelZID = [];
                        icFrom = [];
                        icTo = [];
                        continue;
                    end
                    
                    [commonRelZID, icFrom, icTo] = intersect(relZID_from,relZID_to,'rows');
                    [relRotMat, kappa, deltaTheta, sigmaDeltaTheta] = obj.computeRelativeOrientationConstraint(fromNode, toNode, commonRelZID, icFrom, icTo);
                    
                    if kappa > 0
                        % increment edge count and add edge accordingly
                        obj.relativeRotConstraint(edgeNum).relRotMat = relRotMat;
                        obj.relativeRotConstraint(edgeNum).kappa = kappa;
                        obj.relativeRotConstraint(edgeNum).deltaTheta = deltaTheta;
                        obj.relativeRotConstraint(edgeNum).sigmaDeltaTheta = sigmaDeltaTheta;
                    end
                end
            end
            tCompute = toc(tStart);
            
            fprintf('Done! Number of Edges: %u, Graph Constraints Contruction Time: %f s \n',obj.numEdges, tCompute);
            
        end
        
        function flagig = solveInitialOrientation(obj)
            % Initialize parameters
            n = 2; % SO(n)
            N = obj.numNodes; % number of vertices
            I = obj.edges(:,1);
            J = obj.edges(:,2);
            M = size(I,1);
            H = zeros(n,n,M); % the tensor containing relative rotation measurements
            A = 1; % anchor node
            Ra = obj.nodes(A).DCM; % initialize Ra to true value
            R0 = zeros(n,n,N); % initial guess to 0
            kappa = zeros(M,1);
            
            if M ~= obj.numEdges % this condition should
                error('obj.numEdges should be same as M');
            end
            
            % Construct H and linear system on \delta \theta
            for i = 1:obj.numEdges
                % Build H and Kappa
                H(:,:,i) = obj.relativeRotConstraint(i).relRotMat;
                kappa(i,1) = obj.relativeRotConstraint(i).kappa;
            end
            
            % Construct problem
            problem = buildproblem(n, N, M, I, J, H, kappa, A, Ra);
            
            % Get initial guess by Boumal's method
            [R0, flagig] = initialguess(problem);
            
            if flagig == -1
                fprintf('RFMSLAM: Not all EIGEN Values Converged, I will not use initialGuess based on Eigen Value maximization. \n')
                return;
            end
            
            % first lets extract the chained estimate
            Rchained = zeros(n,n,N);
            for j = 1:obj.numNodes
                Rchained(:,:,j) = obj.nodes(j).DCM;
            end
            
            % now check cost for chained and EIG solution
            l1 = funcost(problem, R0); % EIG
            l2 = funcost(problem, Rchained); % CHAINED
            if l1 < l2 % only if EIG is better than CHAINED update
                % Put back estimated values
                fprintf('RFMSLAM: EIG was better than chained, I will use it. \n')
                for j = 1:obj.numNodes
                    obj.nodes(j).DCM = R0(:,:,j);
                end
            end
            
            
            
        end
        
        function [theta_full, Omega_theta, tCompute] = solveOrientation(obj)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Solve relative orientation optimization problem
            % Output:
            % theta_full: vector of heading estimates
            % Omega_theta: information matrix for theta estimates
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            fprintf('Solve for Orientations...');
            
            % Initialize parameters
            n = 2; % SO(n)
            N = obj.numNodes; % number of vertices
            I = obj.edges(:,1);
            J = obj.edges(:,2);
            M = size(I,1);
            H = zeros(n,n,M); % the tensor containing relative rotation measurements
            A = 1; % anchor node
            Ra = obj.nodes(A).DCM; % initialize Ra to true value
            R0 = zeros(n,n,N); % initial guess to 0
            Rtrue = zeros(n,n,N);
            kappa = zeros(M,1);
            
            if M ~= obj.numEdges % this condition should
                error('obj.numEdges should be same as M');
            end
            
            % We need to form the system:
            % \delta \theta = B \theta + noise
            B_theta = sparse(M,obj.numNodes); % the measurement matrix for relative orientation in 2D
            R_dtheta_inv = sparse(M,M); % the information matrix of relative orientation measurements
            
            % Construct H and linear system on \delta \theta
            for i = 1:obj.numEdges
                % Build H and Kappa
                H(:,:,i) = obj.relativeRotConstraint(i).relRotMat;
                kappa(i,1) = obj.relativeRotConstraint(i).kappa;
                
                % Form the linear system
                % theta_to - theta_form = \delta \theta
                fromNode = I(i);
                toNode = J(i);
                B_theta(i,fromNode) = -1;
                B_theta(i,toNode) = 1;
                R_dtheta_inv(i,i) = 1/(obj.relativeRotConstraint(i).sigmaDeltaTheta)^2; % the measurment covariance is square of sigma
            end
            
            % Right now B_theta and R_dtheta do not have the anchor
            % measurements, we need to add a row on top
            B_theta = [[1,zeros(1,obj.numNodes-1)]; B_theta];
            R_dtheta_inv = blkdiag(1/obj.initialSigma(3)^2,R_dtheta_inv); % the error covariance in knowledge of anchor is extremely small
            
            % Compute the orientation estimate information matrix
            Omega_theta = B_theta'*R_dtheta_inv*B_theta;
            
            % Extract initial guess and truth
            %             for j = 1:obj.numNodes
            %                 R0(:,:,j) = obj.nodes(j).DCM;
            %                 Rtrue(:,:,j) = obj.nodes(j).trueDCM;
            %             end
            
            % Construct problem
            problem = buildproblem(n, N, M, I, J, H, kappa, A, Ra);
            
            % Get initial guess by Boumal's method
            [R0, flagig] = initialguess(problem);
            
            % if eigen values did not converge
            if flagig == -1
                for j = 1:obj.numNodes
                    R0(:,:,j) = obj.nodes(j).DCM;
                end
            end
            % Compute Error in initial guess
            %             initErr = getOrientationError(Rtrue,R0,N);
            %             initRMSE = sqrt(mean(initErr.^2));
            %             fprintf('Initial Guess RMSE = %f degrees \n', initRMSE);
            
            %Solve Problem
            tStart = tic;
            Rmle = synchronizeRotations(problem,R0); % Do optimization
            tCompute = toc(tStart);
            
            % Sometimes, due to numerical issues, the values have very very
            % small imaginary components, enforce only real values used ahead
            Rmle = real(Rmle);
            
            % the vector of orientation measurments,
            % need to output this to be used in full estimation problem
            theta_full = zeros(obj.numNodes,1);
            
            % Put back estimated values
            for j = 1:obj.numNodes
                obj.nodes(j).DCM = Rmle(:,:,j);
                
                theta_full(j,1) = dcm2yaw(obj.nodes(j).DCM);
            end
            
            % Final Error
            %             finalErr = getOrientationError(Rtrue,Rmle,N);
            %             finalRMSE = sqrt(mean(finalErr.^2));
            %             fprintf('Final RMSE = %f degrees \n', finalRMSE);
            fprintf('Done! Optimizations Time: %f s\n', tCompute);
            
        end
        
        function timeToSolve = solveRobotFeaturePositions(obj, theta_full, Omega_theta)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Solve for feature poses. This function
            % should only be called after heading is solved.
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            fprintf('Solving for robot trajectory and feature positions... \n')
            
            relObsDim = 2;
            
            numUniqueLandmarks = size(obj.uniqueLandmarkIDs,2);
            N_d = obj.totalR2FMeasurements; % total number of robot to feature measurements
            N_odo = obj.numNodes-1; % Number of odometery measurements is numposes-1
            n_d = 0; % number of measurements done
            
            d_w_full = zeros(relObsDim*(N_d+N_odo+1),1); % the full stack of global frame measurements
            
            R_d_w_full_inv = sparse(relObsDim*(N_d + N_odo+1),relObsDim*(N_d + N_odo+1)); % the full information matrix of global frame odometery and robot 2 feature measurements
            
            MM = sparse(relObsDim*(N_d+N_odo+1),obj.numNodes); % This is part of the jacobian = \frac{\partial{C^T d_l}}{\partial \theta}
            
            A = sparse(relObsDim*(N_d+N_odo+1),relObsDim*(numUniqueLandmarks+obj.numNodes)); % the A matrix for full llsq
            
            tStart = tic; % we time the process of solving the problem
            
            %% Step 0
            % Add the first pose of robot as known
            % No need to add anything to MM, since first is known constant
            % so jacobian is 0
            d_w_full(1:2,1) = obj.nodes(1).xtrue(1:2,1); % the true position of anchor (first pose)
            A(1:2,1:2) = eye(relObsDim); % the measurement matrix
            R_d_w_full_inv(1:2,1:2) = diag([1/obj.initialSigma(1)^2,1/obj.initialSigma(2)^2]);
            n_d = 1;
            
            %% Step 1
            % Bring in all the robot to robot position odometery constraints
            for i = 1:obj.numNodes-1
                
                % Extract the odometery measurement
                d_l = obj.proprioceptiveOdo(i).mean(1:2,1);
                R_d_l = obj.proprioceptiveOdo(i).cov(1:2,1:2);
                
                % We need DCM' to project from local to global
                CCt = obj.nodes(i).DCM';
                
                % The first node is the anchor, that rotation is a
                % constant, so jacobian is 0.
                if i ==1
                    dCCt_dtheta = CCt*0;
                else
                    ctheta = obj.nodes(i).DCM(1,1);
                    stheta = obj.nodes(i).DCM(1,2);
                    
                    % Compute the jacobian of transpose of DCM w.r.t theta
                    dCCt_dtheta = [-stheta -ctheta;ctheta -stheta];
                end
                
                % transform local to global
                d_w = CCt*d_l;
                
                % Insert into full measurment vector
                d_w_full(n_d*relObsDim+1:n_d*relObsDim+2,1) = d_w;
                
                % Fill in MM with the appropriate jacobians computations
                MM(n_d*relObsDim+1:n_d*relObsDim+2,i) = dCCt_dtheta*d_l;
                
                % Fill in A matrix.
                A(n_d*relObsDim+1:n_d*relObsDim+2,i*relObsDim-1:i*relObsDim) = -eye(relObsDim); % from robot pose
                
                A(n_d*relObsDim+1:n_d*relObsDim+2,(i+1)*relObsDim-1:(i+1)*relObsDim) = eye(relObsDim); % to robot pose
                
                R_d_w_inv = CCt*(R_d_l \ CCt');
                
                R_d_w_full_inv(n_d*relObsDim+1:n_d*relObsDim+2,n_d*relObsDim+1:n_d*relObsDim+2) = R_d_w_inv;
                
                n_d = n_d + 1 ;
            end
            
            %% Step 2
            % Convert each local robot to feature measurement set to global
            % from local coordinate frame, and construct the required
            % matrices and vectors for llsq
            for i = 1:obj.numNodes
                
                % extract the local relative measurement
                d_l = obj.nodes(i).r2f_l;
                R_d_l = obj.nodes(i).R_r2f_l;
                
                % the number of measurements
                numZ = size(obj.nodes(i).z,2);
                
                if numZ == 0
                    warning('There were no local measurements at this pose, I skipped putting it in z full');
                    continue;
                end
                
                % Using kron is a trick to create a block diagonal
                % matrix with repeated blocks, here we repeat the DCM'
                CCt = kron(eye(numZ),obj.nodes(i).DCM');
                
                if i ==1
                    dCCt_dtheta = CCt*0;
                else
                    ctheta = obj.nodes(i).DCM(1,1);
                    stheta = obj.nodes(i).DCM(1,2);
                    
                    % Compute the jacobian of transpose of DCM w.r.t theta
                    dCt_dtheta = [-stheta -ctheta;ctheta -stheta];
                    
                    dCCt_dtheta = kron(eye(numZ),dCt_dtheta);
                end
                % transform local to global
                d_w = CCt*d_l;
                R_d_w_inv = CCt*(R_d_l \ CCt');
                
                % Form the linear system, insert d_w into d_full, insert
                % covariance matrix into R_d_full
                d_w_full(n_d*relObsDim+1:(numZ+n_d)*relObsDim,1) = d_w;
                R_d_w_full_inv(n_d*relObsDim+1:(numZ + n_d)*relObsDim,n_d*relObsDim+1:(numZ +n_d)*relObsDim) = R_d_w_inv;
                
                % Fill in MM with the appropriate jacobians computations
                MM(n_d*relObsDim+1:(numZ+n_d)*relObsDim,i) = dCCt_dtheta*d_l;
                
                % Build up A matrix. A matrix rows have -1,0,1 depending on
                % how the relative measurement is, i.e., from which feature
                % to which feature.
                for j = 1:numZ
                    
                    % relative measurement which goes like
                    % from robot ----> to feature id
                    indxFrom = i;
                    
                    % Find the feature index in the list
                    % of unique landmark ids
                    indxTo = find(obj.uniqueLandmarkIDs == obj.nodes(i).idvfs(j));
                    
                    indxTo = indxTo + obj.numNodes; % in the X vector, features comes after poses
                    
                    % Put in -1 for From
                    A(n_d*relObsDim+j*relObsDim-1:n_d*relObsDim+j*relObsDim,indxFrom*relObsDim-1:indxFrom*relObsDim) = -eye(relObsDim);
                    
                    
                    % Put in +1 for To
                    A(n_d*relObsDim+j*relObsDim-1:n_d*relObsDim+j*relObsDim,indxTo*relObsDim-1:indxTo*relObsDim) = eye(relObsDim);
                end
                
                % increment n_d (it tracks how many measurements we have
                % inserted upto now
                n_d = n_d + numZ;
            end
            
            % Build up the full matrix
            % z = M [l;theta] + noise
            % A2 = [A 0;0 I]
            A2 = blkdiag(A,speye(obj.numNodes));
            z = [d_w_full;theta_full];
            
            R_d_w_full_inv = (R_d_w_full_inv + R_d_w_full_inv')/2;
            
            % Construct the information matrix of z
            Omega_z = [R_d_w_full_inv, -R_d_w_full_inv*MM;...
                -MM'*R_d_w_full_inv, Omega_theta+MM'*R_d_w_full_inv*MM];
            
            % Ensure symmetric Omega_z
            Omega_z = (Omega_z + Omega_z')/2;
            
            % Construct the information matrix of X
            Omega_X = A2'*Omega_z*A2;
            
            % Ensure symmetric Omega_X
            Omega_X = (Omega_X + Omega_X')/2;
            
            % Solve for X
            X = Omega_X \ (A2'*Omega_z*z);
            
            timeToSolve = toc(tStart);
            fprintf('Time to solve full estimation LLSQ problem: %f \n', timeToSolve)
            
            %% Extract Information
            obj.extractPosesFeatures(X, Omega_X);
            
        end
        
        function extractPosesFeatures(obj, X, Omega_X)
            % Function to extract feature pose mean and covariance
            % it is slow right now but accurate.
            fprintf('Started extracting marginals..\n')
            tStart = tic;
            relObsDim = 2;
            
            numFeats = size(obj.uniqueLandmarkIDs,2);
            
            Cov_X = double(inverse(Omega_X));
            
            % Ensure symmetric Covariance
            Cov_X = (Cov_X + Cov_X')/2;
            
            % extract Feature pose estimate covariance
            featPosEst = zeros(2,numFeats);
            featPosEstCov = zeros(2,2,numFeats);
            for f = 1:numFeats
                findx =  (obj.numNodes+f)*relObsDim-1:(obj.numNodes+f)*relObsDim;
                
                % too slow but avoids sparse matrix inversion
                %               [fposest, fposestcov] = extractSubsetMeanCovariance(X,Omega_X, findx);
                
                featPosEst(:,f) = X(findx);
                featPosEstCov(:,:,f) = Cov_X(findx,findx);
            end
            
            % Extract robot pose and covariance
            robPosEst = zeros(3,obj.numNodes);
            robPosEstCov = zeros(3,3,obj.numNodes);
            for p = 1:obj.numNodes
                
                % the positions of my pose x,y,theta in X vector
                xypos = p*relObsDim-1:p*relObsDim;
                thetapos = (obj.numNodes+numFeats)*relObsDim + p;
                
                % the indices of variables we want to extract
                pindx = [xypos thetapos];
                
                %                 [rposest, rposeestcov] = extractSubsetMeanCovariance(X, Omega_X, pindx);
                
                robPosEst(:,p) = X(pindx);
                robPosEstCov(:,:,p) = Cov_X(pindx,pindx);
            end
            
            %% Store Values
            obj.estimatedPose = robPosEst;
            obj.poseCovariance = robPosEstCov;
            obj.estimatedFeatures = featPosEst;
            obj.featCovariance = featPosEstCov;
            
            tCompute = toc(tStart);
            fprintf('Done extracting marginals in %f seconds \n',tCompute)
            
        end
        
        function [relRotMat, kappa, deltaTheta, sigmaDeltaTheta] = computeRelativeOrientationConstraint(obj, fromNode, toNode, commonRelZID, icFrom, icTo)
            
            % get the number of feature to feature constraints with odometery
            nF2FConstraints = size(commonRelZID,1);
            
            % dimension of 1 relative measurement
            relObsDim = 2;
            
            % In 2d we have cos(theta) and sin(theta)
            numDCMParams = 2;
            
            % if odometeric constraint present then this will
            % be set to 1
            flag = 0;
            
            % for successive nodes, there is proprioceptive odometery
            if toNode == fromNode + 1
                % increment number of edges
                flag = 1;
            end
            
            % We have to solve the equation B = Ax +v, v ~ N(0,Q)
            % where
            % B: vector of relative measurements seen at 'from'
            % A: matrix composed of rel measurement seen at 'to'
            % x: vector of unknown dcm params
            if flag == 1 % we have 1 additional constraint from proprioceptive odometery
                Q = zeros(numDCMParams+relObsDim*nF2FConstraints);
                B = zeros(numDCMParams+relObsDim*nF2FConstraints,1);
                A = zeros(numDCMParams+relObsDim*nF2FConstraints,2);
                
                % Get the prop odometery constraint
                dtheta = obj.proprioceptiveOdo(fromNode).mean(3,1); % mean val, fromNode is correct here
                var_dtheta = obj.proprioceptiveOdo(fromNode).cov(3,3); % error cov, fromNode is correct here
                
                % Feed in odometery constraint
                JacobianC = [-sin(dtheta);cos(dtheta)]; % jacobian of c(\theta) = [cos(\theta);sin(\theta)];
                Qodo = JacobianC*var_dtheta*JacobianC';
                Q(1:2,1:2) = Qodo;
                A(1:2,1:2) = [1 0;0 1];
                B(1:2,1) = [cos(dtheta);sin(dtheta)];
                relRotEst = yaw2rotmat(dtheta); % C_init is odometric
            else
                Q = zeros(relObsDim*nF2FConstraints);
                B = zeros(relObsDim*nF2FConstraints,1);
                A = zeros(relObsDim*nF2FConstraints,2);
                
                C_from = obj.nodes(fromNode).DCM;
                C_to = obj.nodes(toNode).DCM;
                
                relRotEst = C_from*C_to'; % = C_p C_q', thats right, from chaining
                if any(any(relRotEst)) == 0
                    error('You did not chain together odometery.')
                end
            end
            
            % Setup the A,B,Q matrices
            % There have to be some rel measurement constraints
            % otherwise odometery has taken care of everything above
            if nF2FConstraints > 0
                
                % extract all local measurements at toNode
                d_l_from = obj.nodes(fromNode).d_l;
                R_d_l_from = obj.nodes(fromNode).R_d_l;
                
                % extract the appropriate constraints
                E_from = zeros(relObsDim*nF2FConstraints, size(d_l_from,1));
                
                for m = 1:nF2FConstraints
                    E_from(relObsDim*m-1:relObsDim*m,relObsDim*icFrom(m)-1:relObsDim*icFrom(m)) = eye(relObsDim);
                end
                
                d_l_c_from = E_from*d_l_from;
                R_d_l_c_from = E_from*R_d_l_from*E_from';
                
                % extract all local measurements at toNode
                d_l_to = obj.nodes(toNode).d_l;
                R_d_l_to = obj.nodes(toNode).R_d_l;
                
                % extract the appropriate constraints
                E_to = zeros(relObsDim*nF2FConstraints, size(d_l_to,1));
                
                for m = 1:nF2FConstraints
                    E_to(relObsDim*m-1:relObsDim*m,relObsDim*icTo(m)-1:relObsDim*icTo(m)) = eye(relObsDim);
                end
                
                d_l_c_to = E_to*d_l_to;
                R_d_l_c_to = E_to*R_d_l_to*E_to';
                
                blkrelRotEst = [];
                
                for aa = 1:nF2FConstraints
                    tempD = d_l_c_to(relObsDim*aa-1:relObsDim*aa,1);
                    
                    A(flag*numDCMParams+relObsDim*(aa-1)+1:flag*numDCMParams+aa*relObsDim,:) = [tempD(1) -tempD(2);tempD(2) tempD(1)];
                    blkrelRotEst = blkdiag(blkrelRotEst,relRotEst);
                end
                
                B(flag*numDCMParams+1:end,1) = d_l_c_from;
                
                Q(flag*numDCMParams+1:end,flag*numDCMParams+1:end) = R_d_l_c_from + blkrelRotEst*R_d_l_c_to*blkrelRotEst';
                
            end
            
            % You HAVE to enforce symmetric
            Q = (Q+Q')/2; % Do not remove this line Saurav
            
            if ~isreal(Q)
                error('R_d_l was made Symmetric PD, check why Q is not real?')
            end
            
            % Make it Positive Definite
            Q = nearPD(Q); % Do not remove this line Saurav
            
            % Solve linear least squares
            X = lscov(A,B,Q);
            
            % Compute normalization factor
            eta = norm(X,2);
            
            J = (1/eta^3)*[X(2)^2, -X(1)*X(2);-X(1)*X(2), X(1)^2];
            
            % Normalize to enforce rotation matrix constraint
            X = X / eta;
            
            if ~isreal(X)
                error('Rotations are imaginary, not possible')
            end
            
            Omega_X_nonnorm =  A'*(Q\A);
            
            % The inversion requires PD matrix
            Omega_X_nonnorm = nearPD(Omega_X_nonnorm);
            
            CovX =  J*(Omega_X_nonnorm \ J');
            
            % If error covariance comes to inf then reject this computation
            if any(any(isinf(CovX)))
                relRotMat = -1;
                kappa = -1;
                deltaTheta = -1;
                sigmaDeltaTheta = -1;
                return;
            end
            
            % Enfore real Symmetric
            CovX = real((CovX + CovX')/2);
            
            CovX = nearPD(CovX);
            
            % X is the vector of c_11,c_21
            % chain the relative to previous
            % initialize relative rotation matrix
            relRotMat = [X(1) -X(2);...
                X(2) X(1)];
            
            % Get the delta angle and related cov
            deltaTheta = rotmat2yaw(relRotMat);
            
            costheta = X(1);
            sintheta = X(2);
            
            % Jacobian of atan2(X(2), X(1)) w.r.t X
            J2 = [-sintheta costheta];
            
            sigmaDeltaTheta = sqrt(J2*CovX*J2');
            
            if isnan(sigmaDeltaTheta) || ~isreal(sigmaDeltaTheta)
                error('Cannot be NAN or imaginary');
            end
            
            kappa = 1/abs(sigmaDeltaTheta);
            
            % kappa goes too high, cap it, otherwise it doesn't let
            % optimizer work properly
            if kappa >= 3.4e+09 % 3.4e+07 is the value of kappa for 0.01 degree std dev
                kappa = 3.4e+09;
            end
            
        end
        
        function loadGraphFromFile(obj,fname)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Insert data into graph by reading
            % a text file.
            %
            % Input:
            % baseDir: path to directory containig file
            % fname: the name of file
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            fprintf('Reading slam data from input file...\n')
            
            % open the file
            fid = fopen(fname);
            
            % the total number of nodes
            numnodes = 0;
            
            % the total number of proprioceptive odometery edges
            numpropodoedges = 0;
            
            % total number of robot to feature measurements
            numr2fz = 0;
            
            % read it line by line
            while true
                
                % read line
                tline = fgetl(fid);
                
                % check if it has anything in it
                if ~ischar(tline);
                    break;%end of file
                end
                
                % if it had something, lets see what it had
                % Scan the line upto the first space
                % we will identify what data type this is
                str = textscan(tline,'%s %*[ ]');
                
                if strcmp(str{1},'VERTEX2')
                    
                    datastr = textscan(tline,'%s %d %f %f %f');
                    numnodes = datastr{2};
                    
                    % store the data in each node
                    obj.nodes(numnodes).DCM = zeros(2,2); % initialize an unknown DCM
                    
                    % First node is anchor node (known pose)
                    if numnodes == 1
                        xtrue = [datastr{3};datastr{4};datastr{5}];
                        obj.nodes(numnodes).trueDCM = yaw2rotmat(xtrue(3))'; % get the true DCM
                        obj.nodes(numnodes).DCM = yaw2rotmat(xtrue(3,1))'; % transpose of rot mat
                        obj.nodes(numnodes).xtrue = xtrue; % the true pose
                    end
                    
                    obj.initialGuessTraj = [obj.initialGuessTraj, [datastr{3};datastr{4};datastr{5}]];
                    
                elseif strcmp(str{1}, 'EDGE2')
                    
                    datastr = textscan(tline,'%s %d %d %f %f %f %f %f %f %f %f %f');
                    
                    fromnode = datastr{2};
                    propOdoVal = [datastr{4};datastr{5};datastr{6}];
                    propOdoCov = [datastr{7}, datastr{8}, datastr{9};...
                        datastr{8}, datastr{10}, datastr{11};...
                        datastr{9}, datastr{11}, datastr{12}];
                    
                    % make sure cov is symmetric
                    assert(issymmetric(propOdoCov));
                    
                    obj.insertPropOdoEdge(propOdoVal, propOdoCov, fromnode);
                    
                    numpropodoedges = numpropodoedges +1;
                    
                elseif strcmp(str{1},'BR')
                    datastr = textscan(tline,'%s %d %d %f %f %f %f');
                    
                    fromnode = datastr{2};
                    
                    % we subtract because in graph file, we added total node number
                    % to featid to help gtsam get separate ids for features nodes
                    % and pose nodes
                    tofeat = datastr{3} - numnodes;
                    
                    rb = [datastr{5};datastr{4}];
                    sigma = [datastr{7};datastr{6}];
                    rb_cov = diag(sigma.^2);
                    
                    obj.nodes(fromnode).idvfs = [obj.nodes(fromnode).idvfs, tofeat]; % ids of visible feats
                    obj.nodes(fromnode).z = [obj.nodes(fromnode).z, rb]; % range bearing
                    obj.nodes(fromnode).R_z = blkdiag(obj.nodes(fromnode).R_z, rb_cov); % measurement noise covariance
                    
                    numr2fz = numr2fz + 1;
                else
%                     error('Unrecognized data type in file.')
                end
                
            end
            
            % close file
            fclose(fid);
            
            assert(numpropodoedges + 1 == numnodes);
            
            obj.numNodes = double(numnodes);
            obj.totalR2FMeasurements = numr2fz;
            
            fprintf('    Total nodes: %d \n', numnodes)
            fprintf('    Total odometery edges: %d \n', numpropodoedges)
            fprintf('    Tota range-bearing measurements: %d \n', numr2fz);
            fprintf('Done reading input file.\n')
            
        end
        
        function draw(obj)
            
            if isempty(obj.estimatedFeatures)
                return;
            end
            
            X = obj.estimatedFeatures(1,:);
            Y = obj.estimatedFeatures(2,:);
            
            N = 50;% number of points in ellipse drawing
            confidence = 0.9545; % draw the 95% (2-sigma) confidence ellipse
            
            pcov = [];
            
            for i = 1:size(obj.estimatedFeatures,2)
                covMat = nearPD(obj.featCovariance(2*i-1:2*i,2*i-1:2*i));
                ptemp = error_ellipse('C',covMat,'mu',obj.estimatedFeatures(:,i),'conf',confidence,'scale',1,'N', N);
                pcov = [pcov ptemp];
            end
            
            set(obj.fHandle.xguess, 'xdata', obj.initialGuessTraj(1,:), 'ydata', obj.initialGuessTraj(2,:));
            set(obj.fHandle.xf, 'xdata', X, 'ydata', Y)
            set(obj.fHandle.cov, 'xdata', pcov(1,:), 'ydata', pcov(2,:))
            set(obj.fHandle.xr, 'xdata', obj.estimatedPose(1,:), 'ydata', obj.estimatedPose(2,:))
            
        end
        
    end
    
end