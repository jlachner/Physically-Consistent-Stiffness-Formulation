%% Cleaning Up + Environment Setup
clear; close all; clc;

% Simulation settings
simTime = 5;        % Total simulation time
t = 0;
dt = 0.005;         % Time-step of simulation
n_steps = simTime / dt; % Number of steps

% Initialize Robot
robot = iiwa14('low');
robot.init();

%% Create Animation
anim = Animation('Dimension', 3, 'xLim', [-0.1, 1], 'yLim', [-0.1, 1], 'zLim', [-0.2, 0.8]);
anim.init();
anim.attachRobot(robot);

%% Initial Kinematics
q = deg2rad([63.42, 72.31, -32.80, -98.44, 2.72, -28.69, -63.11]');
q = [1.3184, 0.9462, -0.4382, -1.9943, -0.0564, -0.0986, -3.7078]';
dq = zeros(robot.nq, 1);

robot.updateKinematics(q);
anim.update(0);

%% Initial Transformation
pointPos = [0, 0, 0.275];
H_ini = robot.getForwardKinematics(q, 'bodyID', 7, 'position', pointPos);
R_tilt = H_ini(1:3, 1:3);
R_last = R_tilt;
center = H_ini(1:3, 4); % Center of circular motion

%% Plane Setup
plane_size = 0.1;
[X, Y] = meshgrid(linspace(-plane_size/2, plane_size/2, 10), ...
                   linspace(-plane_size/2, plane_size/2, 10));
Z = zeros(size(X)); % Flat plane in local coordinates
plane_points = [X(:), Y(:), Z(:), ones(numel(X),1)]'; 

%% Circular Trajectory Setup
radius = 0.09; % Adjusted radius
rotation_angle = deg2rad(-115); % Rotation about tangent direction

% Generate theta values for half-circle and back
theta_vals = [linspace(0, pi, n_steps/2), linspace(pi, 0, n_steps/2)];
theta_ini = theta_vals(1);

% Plot circular trajectory
plot3(center(1) + radius * cos(theta_vals), ...
      center(2) + radius * sin(theta_vals), ...
      center(3) * ones(size(theta_vals)), 'k--', 'LineWidth', 1);

%% End-Effector Coordinate System
hg_cs = hgtransform;
[VFC{1:3}] = func_create_VFC_data('Koordinatensystem', 12);
VFC{1} = VFC{1} / 7;
[V, F, C] = VFC{:};
patchTCPeef = patch('Faces', F, 'Vertices', V, 'FaceVertexCData', C, 'FaceC', 'flat', ...
    'EdgeColor', 'none', 'Parent', hg_cs, 'FaceAlpha', 1, 'Tag', 'TCP');

% Set initial transformation
H_ini = eye(4);
H_ini(1:3, 1:3) = R_tilt; % Tilted dynamic rotation
H_ini(1:3, 4) = center + radius * [cos(theta_ini); sin(theta_ini); 0];
set(hg_cs, 'Matrix', H_ini);

%% Arrow Parameters
hg_arrow = hgtransform; % Arrow transformation
arrow_length = 0.1;
arrow_scale = 12; % Adjust arrow head size

arrow = quiver3(0, 0, 0, arrow_length, 0, 0, 'r', 'LineWidth', 2, ...
                'MaxHeadSize', arrow_scale, 'Parent', hg_arrow);

%% Plane Transformation
hg_plane = hgtransform;
plane_patch = surf(reshape(plane_points(1,:), size(X)), ...
                   reshape(plane_points(2,:), size(Y)), ...
                   reshape(plane_points(3,:), size(Z)), ...
                   'FaceColor', 'cyan', 'FaceAlpha', 0.6, 'EdgeColor', 'k', ...
                   'Parent', hg_plane);

%% Record joint configurations
pause(2);

q_record = zeros( robot.nq, n_steps );
t_record = zeros( 1, n_steps );

%% Simulation Loop
for step = 1:n_steps
    theta = theta_vals(step);

    % Compute new position on circular trajectory
    position = center + radius * [cos(theta); sin(theta); 0];

    % Compute tangent (movement direction)
    x_new = [-sin(theta); cos(theta); 0];
    x_new = x_new / norm(x_new);

    % Compute normal (z-axis, pointing to center)
    z_new = center - position;
    z_new = z_new / norm(z_new);

    % Compute perpendicular direction (y-axis)
    y_new = cross(z_new, x_new);
    y_new = y_new / norm(y_new);

    % Ensure orthogonality
    x_new = cross(y_new, z_new);

    % Construct base rotation matrix
    R_base = [x_new, y_new, z_new];

    % Rotation about movement direction (Rodrigues' rotation)
    c = cos(rotation_angle);
    s = sin(rotation_angle);
    v = 1 - c;
    u = x_new;
    
    R_about_x = [u(1)^2*v + c,      u(1)*u(2)*v - u(3)*s,  u(1)*u(3)*v + u(2)*s;
                 u(2)*u(1)*v + u(3)*s,  u(2)^2*v + c,      u(2)*u(3)*v - u(1)*s;
                 u(3)*u(1)*v - u(2)*s,  u(3)*u(2)*v + u(1)*s,  u(3)^2*v + c];

    % Compute final rotation
    R_final = R_about_x * R_base;

    % Construct transformation matrices
    H_plane = eye(4);
    H_plane(1:3, 1:3) = R_final;
    H_plane(1:3, 4) = position;

    H_arrow = eye(4);
    H_arrow(1:3, 1:3) = R_final;
    H_arrow(1:3, 4) = position;

    % Apply transformations
    set(hg_plane, 'Matrix', H_plane);
    set(hg_arrow, 'Matrix', H_arrow);
    set(hg_cs, 'Matrix', H_arrow);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Inverse kinematicis
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Update Robot Kinematics
    H = robot.getForwardKinematics(q, 'bodyID', 7, 'position', pointPos);
    p = H(1:3, 4);
    p_new = position;

    % Compute linear velocity
    del_p = (p_new - p);
    dp = del_p / dt;

    % Compute angular velocity
    R_rel = R_final * R_last';
    axang = rotm2axang(R_rel);
    w = (axang(4) / dt) * axang(1:3)';

    % Compute body twist
    tw = [dp; w];

    % Inverse Kinematics
    J = robot.getHybridJacobian(q);
    M = robot.getMassMatrix(q);
    M_inv = M \ eye(size(M));
    k = 0.01;
    Lambda_inv = J * M_inv * J' + (k^2) * eye(6, 6);
    Lambda = Lambda_inv \ eye(size(Lambda_inv));
    J_inv = M_inv * J' * Lambda;

    % Compute new joint configuration
    dq_0 = J_inv * tw;
    q = q + (dq_0 * dt);

    % Store new joint configuration
    q_record( :, step ) = q;
    t_record( step ) = t;

    % Update states
    q_last = q;
    p_last = p;
    R_last = R_final;

    % Update Robot
    robot.updateKinematics(q);

    % Update animation
    t = t + dt;
    anim.update(t);
end

anim.close( );


%% Can Run Imitation Learning, but after checking the code works

%IMIT_LEARN_LATER

% Apply Savitzky-Golay Filter to smooth the joint data
polynomialOrder = 3; % Typically a low-order polynomial
frameSize = 11; % Must be an odd number, adjust based on your noise level

% Smooth each joint trajectory
q_record_smoothed = sgolayfilt(q_record, polynomialOrder, frameSize, [], 2);

% Plot the original smoothed data for comparison (optional)
figure;
plot(t_record, q_record_smoothed, 'LineWidth', 2);
title('Smoothed Original Joint Trajectories');
xlabel('Time (s)');
ylabel('Joint Angles (rad)');

%% Save smoothed data as csv for replay
csvwrite('q_recorded_smoothed.csv', q_record_smoothed);

%IMIT_LEARN_LATER_END

%% Save raw data as csv for comparison
csvwrite('q_recorded.csv', q_record);





