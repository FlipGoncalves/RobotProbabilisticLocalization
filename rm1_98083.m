% ===============================================================================================
% Encyclopedia of EEE -- Robot Localization: An Introduction
%
%           Shoudong Huang, Gamini Dissanayake
%
%           Centre for Autonomous Systems
%           Faculty of Engineering and Information Technology
%           University of Technology, Sydney
%           NSW 2007, Australia
% 
% MATLAB code for the examples: Version: 1.0
% ===============================================================================================
% 
% Copyright (C) 2016 Shoudong Huang 
% University of Technology, Sydney, Australia
% 
% Author:  Shoudong Huang     -- Shoudong.Huang@uts.edu.au
%          
% Please contact Shoudong Huang {Shoudong.Huang@uts.edu.au} if you have any questions/comments about the code.
%
% Modified by v.santos (vitor@ua.pt, March 2023, 2024): Version 3.0
% 
% ===============================================================================================
% Generate data and perform EKF localization this code generates data for EKF localization and then run EKF
%
% Data generated include: 
% landmark data -- landmarkxy
% control data -- control_input_mea
% observation data -- obs_range_bearing
% true robot pose data -- xstate_true (for comparison)
%
% Shoudong Huang, 2016 April
% ===============================================================================================

function rm1_98083(N, Dt, r, L, Vn, Wn, V)
    % N - number of beacons (4)
    % Dt - sensors sampling time interval (1 s)
    % r - radius of the robots wheels (0.15 m)
    % L - separation/distance of the wheels according to the kinematic model (1 m)
    % Vn - uncertainty (sigma) in the linear velocity to be imposed on the robot (input) (0.1 m/s)
    % Wn - uncertainty (sigma) in the angular velocity to be imposed on the robot (input) (0.1 m/s)
    % V - desired average linear velocity along trajectory (5 m/s)
    
    close all
    clc

    % add lib
    addpath 'lib'

    % In case nothing is passed by in the function arguments
    % We need to make sure there are default values to some important variables
    if nargin < 1
        N = 4;
    end
    if nargin < 2
        Dt = 1;
    end
    if nargin < 3
        r = 0.15;
    end
    if nargin < 4
        L = 1;
    end
    if nargin < 5
        Vn = 0.1;
    end
    if nargin < 6
        Wn = 0.1;
    end
    if nargin < 7
        V = 5;
    end

    % observation: range, bearing
    sig_r   = 0.1;
    sig_phi = 0.05;

    % get all the beacons beforehand
    B = BeaconDetection(N);
    X = vertcat(B.X);
    Y = vertcat(B.Y);
    a = vertcat(B.a);

    % landmark format: ID, x, y distance, angle
    % control inputs setting: time_step, linear velocity, turnrate (angular velocity)
    landmarkxy = zeros(size(X, 1), 5);
    P = [0 0];                          % initial position for distance measure
    traj_pos_x = [0];                   % trajectory x coordinate
    traj_pos_y_straight = [0];          % straight trajectory y coordinate  ->  used only for chart
    for i=1:size(X, 1)
        distance = sqrt(power(X(i) - P(1), 2) + power(Y(i) - P(2), 2));         % calculate distance from last beacon to new beacon
        landmarkxy(i,:) = [i, X(i), Y(i), distance, a(i)];                      % update landmark array
        n_steps = round(distance / V / Dt, TieBreaker="tozero") + 1;            % calculate number of steps between beacons
        lins = linspace(P(1), X(i), n_steps);                                   % get x poses for the trajectory
        lins_straight = linspace(P(2), Y(i), n_steps);                          % get y poses for the straight trajectory
        traj_pos_x = [traj_pos_x lins(2:end)];                                  % update trajectory x poses
        traj_pos_y_straight = [traj_pos_y_straight lins_straight(2:end)];       % update straight trajectory y poses
        P = [X(i) Y(i)];                                                        % update last beacon position
    end

    % calculate the trajectory using the Path by Hermite polynomial interpolation (pchip)
    landmarks_x = [0; landmarkxy(:,2)];                         % get landmarks x poses (with initial position)
    landmarks_y = [0; landmarkxy(:,3)];                         % get landmarks y poses (with initial position)
    traj_pos_y = pchip(landmarks_x, landmarks_y, traj_pos_x);

    % draw the straight trajectory and the trajectory using the Path by Hermite polynomial interpolation (pchip)
    figure(1)
    subplot(2,1,1);
    axis([-1 max(landmarkxy(:,2))+20 -1 max(landmarkxy(:,3))+20])
    hold on
    plot(traj_pos_x,traj_pos_y_straight,'LineWidth',2, 'Color', 'r', 'LineStyle', '--')
    plot(landmarkxy(:,2),landmarkxy(:,3),'k*','MarkerSize',8);
    text(landmarkxy(:,2)+0.2,landmarkxy(:,3),num2str(landmarkxy(:,1)),'fontweight','bold','fontsize',14)
    title("Beacons and the linearised path")
    grid on

    subplot(2,1,2);
    axis([-1 max(landmarkxy(:,2))+20 -1 max(landmarkxy(:,3))+20])
    hold on
    plot(traj_pos_x,traj_pos_y,'LineWidth',2, 'Color', 'g')
    plot(landmarkxy(:,2),landmarkxy(:,3),'bo','MarkerSize',8);
    text(landmarkxy(:,2)+0.2,landmarkxy(:,3),num2str(landmarkxy(:,1)),'fontweight','bold','fontsize',14)
    for i = 1:numel(traj_pos_x)
        plot([traj_pos_x(i), traj_pos_x(i)], [0, traj_pos_y(i)], '--k'); % Plot a dashed black line from each point to the x-axis
    end
    title("V = "+ V +" m/s ; Dt = "+Dt+" s ; Dd = "+V*Dt+" m", "Path by Hermite polynomial interpolation (pchip)")
    grid on

    % calculate the control input poses for the two variations of mobile robot: Diferencial Drive (DD) and Tricycle (TRI)
    titles = ["DD", "TRI"];
    for type=1:2
        % open file to write the poses
        fileID = fopen(titles(type)+'_98083.txt','w');
        fprintf(fileID, "%5s, %5s, %5s\r\n", ["VR", "VL", "W"]);
    
        % initialize variables
        th = 0;
        control_input_true = [];
        for step=1:size(traj_pos_y, 2)-1
            % get position poses
            x = traj_pos_x(step);
            y = traj_pos_y(step);
    
            % get next position poses
            x_next = traj_pos_x(step+1);
            y_next = traj_pos_y(step+1);
            
            % calculate orientation based on the last orientation
            temp_th = th;
            th = atan2(y_next - y, x_next - x);
            new_th = th - temp_th;
    
            % calculate x and y difference from two points
            X = x_next - x;
            Y = y_next - y;
        
            % angular movement
            % add pose
            control_input_true = [control_input_true; step 0 new_th];
            fprintf(fileID, "%5.3f, %5.3f, %5.4f\r\n", [0 0 new_th]);
        
            % linear movement
            w3 = 0;
            % DD mobile robot
            if type == 1
                [VR, VL] = invkinDDxy(X, Y, L, Dt, 0, 0);   % get DD inverse kinematics
                w1 = VL/r;                                  % calculate angular velocity of left wheel
                aw2 = VR/r;                                 % calculate angular velocity of right wheel
            % TRI mobile robot
            else                
                w1 = V * cos(new_th) / r - sin(new_th);         % calculate angular velocity of steering wheel
                aw2 = new_th / L;                               % calculate alpha
            end
    
            % calculate x and y velocities for the mobile robot
            [Vx, Vy, w] = localvels(type, r, L, w1, aw2, w3);
            R = orm(new_th);
            W = inv(R) * [Vx Vy w]';
            control_input_true = [control_input_true; step W(1)*cos(new_th) + W(2)*sin(new_th) 0];
            fprintf(fileID, "%5.3f, %5.3f, %5.4f\r\n", [VR VL 0]);
        end
        fclose(fileID);
    
        % number of motion steps (starts from time step 0)
        num_steps = size(control_input_true, 1);
    
        % generating measured control inputs by adding noises (for EKF to use)
        control_input_mea      = control_input_true;
        noises_v               = randn(num_steps,1) * Vn;
        noises_omega           = randn(num_steps,1) * Wn;
        control_input_mea(:,2) = control_input_mea(:,2) + noises_v;
        control_input_mea(:,3) = control_input_mea(:,3) + noises_omega;
        
        % generate ground true robot poses: pose ID, x, y, phi
        xstate_true = [0, zeros(1,3)];              % pose at time 0
        
        for i=1:num_steps
            control_i     = control_input_true(i,2:3);
            control_noise = [0;0];
            xstatet1      = motionmodel(xstate_true(end,2:4),control_i,control_noise,Dt);
            xstate_true   = [xstate_true; i xstatet1];            % accumulate the xstate Ground Truth
        end
        
        % Define the noise covariances for EKF
        Q = [ Vn^2  0
              0     Wn^2 ];
        % for one landmark observation
        R_i = [ sig_r^2  0
                0        sig_phi^2 ];
        
        % for storing the results
        xstate_EKF = [0, zeros(1,3)];       % pose at time 0
        P_EKF      = 0.01*eye(3);           % initial covariance matrix
        
        % initial estimate in EKF
        for step = 1:num_steps
            
            % EKF estimate at time t
            xstate_t = xstate_EKF(end,2:4)';
            P_t = P_EKF(end-2:end,:);

            % get closest beacons
            B = BeaconDetection(N, xstate_t);           % Get landmarks
            X = vertcat(B.X);                           % Get X coordinates
            Y = vertcat(B.Y);                           % Get Y coordinates
            d = vertcat(B.d);                           % Get distances

            % get id of the two most closest landmarks
            best_d = sort(d);
            landmark1_id = find(sum(ismember(d, best_d(1)), 2) >= 1);
            landmark2_id = find(sum(ismember(d, best_d(2)), 2) >= 1);

            % get closest two landmarks
            landmark1 = [X(landmark1_id) Y(landmark1_id)];    
            landmark2 = [X(landmark2_id) Y(landmark2_id)];  
            
            % get id of the landmarks
            landmark1_id = find(sum(ismember(landmarkxy, landmark1), 2) >= 1);
            landmark2_id = find(sum(ismember(landmarkxy, landmark2), 2) >= 1);  
    
            % observation noises
            noise_r      = randn * sig_r;
            noise_phi    = randn * sig_phi;
            sensor_noise = [noise_r noise_phi];
            
            % range-bearing to one landmark
            z1 = sensormodel(landmark1, xstate_true(step+1,2:4), sensor_noise);
            
            % range-bearing to another landmark
            z2 = sensormodel(landmark2, xstate_true(step+1,2:4), sensor_noise);
        
            % control input at time t
            control_t= control_input_mea(step,2:3);
            % observation data at time t+1
            obs_t1 = [landmark1_id z1 landmark2_id z2];
            
            % discretization time interval
            R = [R_i,zeros(2,2); zeros(2,2),R_i]; % because observing two landmarks each step
            % using ekf function
            [xstateT1_T1, PT1_T1] = ekf(xstate_t,P_t,control_t,obs_t1,landmarkxy,Dt,Q,R);
            
            % update
            xstate_EKF = [xstate_EKF; step, xstateT1_T1];
            P_EKF = [P_EKF; PT1_T1];
        end
        
        error_xstate = xstate_EKF - xstate_true;
    
        % draw the robot pose using EKF
        figure(type+1)
        title(titles(type), "Blue -> Estimated Pose / Red -> Real Pose")
        hold on
        grid on
        axis([-1 max(landmarkxy(:,2))+20 -1 max(landmarkxy(:,3))+20])
        plot(landmarkxy(:,2),landmarkxy(:,3),'k*','MarkerSize',14);
        text(landmarkxy(:,2)+0.2,landmarkxy(:,3),num2str(landmarkxy(:,1)),'fontweight','bold','fontsize',14)

        arrow_length=0.3;
        for i=0:num_steps
            % get the xy covariance
            uncer_p = P_EKF(i*3+1:i*3+2, 1:2);
            
            % get x and y uncertainty
            uncer_x = xstate_EKF(i+1,2);
            uncer_y = xstate_EKF(i+1,3);
            % by wangzhan, make it large on purpose, not now
            CV = GetCov(uncer_p,uncer_x,uncer_y);
            plot(CV(1,:), CV(2,:), '-b');
            
            plot(xstate_EKF(i+1,2),xstate_EKF(i+1,3),'bo','linewidth',2);

            % draw the robot heading
            dx = arrow_length*cos(xstate_EKF(i+1,4));
            dy = arrow_length*sin(xstate_EKF(i+1,4));
            quiver(xstate_EKF(i+1,2),xstate_EKF(i+1,3), dx, dy, 0, 'Color', 'b','linewidth',1.2)
            
            % draw the true robot poses for comparison
            plot(xstate_true(i+1,2),xstate_true(i+1,3),'ro','linewidth',1);
            
            dx = arrow_length*cos(xstate_true(i+1,4));
            dy = arrow_length*sin(xstate_true(i+1,4));
            quiver(xstate_true(i+1,2),xstate_true(i+1,3), dx, dy, 0, 'Color', 'r','linewidth',1.2)
        end
    end

    % create file
    filLoc = fopen('loc_98083.txt', 'w');
    fprintf(filLoc, "%5s, %5s, %5s\r\n", ["X", "T", "Th"]);
    for i=1:size(xstate_EKF, 1)
        fprintf(filLoc, "%5.3f, %5.3f, %5.4f\r\n", xstate_EKF(i,2:end));
    end
    fclose(filLoc);
end