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
    clear
    clc

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
    end    if nargin < 5
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

    B = BeaconDetection(N);

    % landmark setting: ID, x, y distance, angle
    % control inputs setting: time_step, linear velocity, turnrate (angular velocity)
    landmarkxy = zeros(N, 5);
    X = vertcat(B.X);
    Y = vertcat(B.Y);
    d = vertcat(B.d);
    a = vertcat(B.a);
    control_input_true = [];
    th = 0;
    P = [0 0];
    for i=1:N
        distance = sqrt(power(X(i) - P(1), 2) + power(Y(i) - P(2), 2));
        landmarkxy(i,:) = [i, X(i), Y(i), distance, a(i)];
        n_steps = round(distance / V / Dt, TieBreaker="tozero") + 1;
        for step=1:n_steps
            if step == 1
                temp = th;
                th = atan2(Y(i) - P(2), X(i) - P(1));
                control_input_true = [control_input_true; 1 0 th - temp];
            else
                control_input_true = [control_input_true; 1 V*Dt 0];
            end
        end
        P = [X(i) Y(i)];
    end

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
    
    % range and bearing observations: time_step ID1 r1 phi1 ID2 r2 phi2
    P = [0 0];
    obs_range_bearing = [];
    for i=1:num_steps
        % Get landmarks
        B = BeaconDetection(N, P);
        X = vertcat(B.X);
        Y = vertcat(B.Y);

        landmark1 = [X(1) Y(1)];
        landmark2 = [X(2) Y(2)];
        
        landmark1_id = find(sum(ismember(landmarkxy, landmark1), 2) >= 1);
        landmark2_id = find(sum(ismember(landmarkxy, landmark2), 2) >= 1);

        % observation noises
        noise_r      = randn * sig_r;
        noise_phi    = randn * sig_phi;
        sensor_noise = [noise_r noise_phi];
        
        % range-bearing to one landmark
        z1 = sensormodel(landmark1, xstate_true(i+1,2:4), sensor_noise);
        
        % range-bearing to another landmark
        z2 = sensormodel(landmark2, xstate_true(i+1,2:4), sensor_noise);
        
        % store the obs data
        obs_range_bearing = [obs_range_bearing; i landmark1_id z1 landmark2_id z2];
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
    
        % control input at time t
        control_t= control_input_mea(step,2:3);
        % observation data at time t+1
        obs_t1 = obs_range_bearing(step,2:end);
        
        % discretization time interval
        R = [R_i,zeros(2,2); zeros(2,2),R_i]; % because observing two landmarks each step
        % using ekf function
        [xstateT1_T1, PT1_T1] = ekf(xstate_t,P_t,control_t,obs_t1,landmarkxy,Dt,Q,R);
        
        % update
        xstate_EKF = [xstate_EKF; step, xstateT1_T1];
        P_EKF = [P_EKF; PT1_T1];
    end
    
    error_xstate = xstate_EKF - xstate_true;
    
    % draw the estimated robot poses and uncertainty ellipses
    figure(1)
    arrow_length=0.3;
    
    axis([-1 max(landmarkxy(:,2))+20 -1 max(landmarkxy(:,3))+20])
    hold on
    
    plot(landmarkxy(:,2),landmarkxy(:,3),'k*','MarkerSize',14);
    text(landmarkxy(:,2)+0.2,landmarkxy(:,3),num2str(landmarkxy(:,1)),'fontweight','bold','fontsize',14)
    grid on
    
    for i=0:num_steps
        uncer_p = P_EKF(i*3+1:i*3+2, 1:2);            % get the xy covariance
        
        uncer_x = xstate_EKF(i+1,2);
        uncer_y = xstate_EKF(i+1,3);
        CV = GetCov(uncer_p,uncer_x,uncer_y);         % by wangzhan, make it large on purpose, not now
        plot(CV(1,:), CV(2,:), '-b');
        
        plot(xstate_EKF(i+1,2),xstate_EKF(i+1,3),'bo','linewidth',2);
    
        % draw the robot heading
        dx = arrow_length*cos(xstate_EKF(i+1,4));
        dy = arrow_length*sin(xstate_EKF(i+1,4));
        quiver(xstate_EKF(i+1,2),xstate_EKF(i+1,3), dx, dy, 0, 'Color', 'b','linewidth',1.2)
        
        % draw the true robot poses for comparison
        plot(xstate_true(i+1,2),xstate_true(i+1,3),'ro','linewidth',2);
        
        dx = arrow_length*cos(xstate_true(i+1,4));
        dy = arrow_length*sin(xstate_true(i+1,4));
        quiver(xstate_true(i+1,2),xstate_true(i+1,3), dx, dy, 0, 'Color', 'r','linewidth',1.2)
        
    end
end