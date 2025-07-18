%% Clean up

clear all; close all; clc;

%% Parse data

raw_data = parse_txt( 'print_joint_val.txt', 0 );

% Read the time (with offset) and joint-trajectory data
t_record = raw_data( :, 1 )' - raw_data( 1, 1 );
q_record = raw_data( :, 2:8 )';
dq_record = raw_data( :, 9:end )';

%% Figure config
fig_config( 'fontSize', 20, 'markerSize', 10 );

% Plot the time vs q graph
% f = figure( ); a = axes( 'parent', f );
% plot( t_record, q_record );

%% Run Explicit to double-check the movement
% Make sure that you have Explicit-MATLAB to replay the code

% Robot to display original movement
robot = iiwa14( 'high' );
robot.init( );

% Robot to display movement that is shifted along -z
robot2 = iiwa14( 'high' );
robot2.init( );

% Initialize animation and attach robots
Nt = length( t_record );
dt = 0.005;

anim = Animation( 'Dimension', 3, 'xLim', [-0.2,1.2], ...
    'yLim', [-0.7,0.7], 'zLim', [-0.6,0.8], 'isSaveVideo', false, 'VideoSpeed', 1.0 );
anim.init( );
anim.attachRobot( robot );
anim.attachRobot( robot2 );

%% Simulate original movement
% for i = 1:Nt
%
%     % Read out time and recorded joint configuration
%     t = t_record( i );
%     q = q_record( :, i );
%
%     robot.updateKinematics( q );
%     anim.update( t );
%
% end
%
% anim.close( );

%% Simulate new movement

% Initialize
q_record_shift = zeros( robot.nq, Nt );

q_last = q_record( :, 1 );

H = robot.getForwardKinematics( q_last );
p_last = H( 1:3, 4 );
del_z = -0.04;                          % SET THIS PARAMETER TO MODIFY OFFSET
p_last( 3 ) = p_last( 3 ) + del_z;
R_last = H( 1:3, 1:3 );

% Runtime Baby!
for i = 1:Nt

    % Read out time and recorded joint configuration
    t = t_record( i );
    q = q_record( :, i );

    % New p
    H = robot.getForwardKinematics( q );
    p = H( 1:3, 4 );
    p_new = p;
    p_new( 3 ) = p_new( 3 ) + del_z;

    % Linear velocity due to translation
    del_p = ( p_new - p );
    dp = del_p / dt;

    % Angular velocity due to rotation
    R = H( 1:3, 1:3 );
    R_rel = R * R_last';
    axang = rotm2axang( R_rel );
    u = axang( 1:3 );
    theta = axang( 4 );
    w = theta / dt * u';

    % Resulting body twist
    tw = [ dp; w ];

    % Inverse kinematics, based on Least-Square Dynamic Consitent Jacobian
    J = robot.getHybridJacobian( q );
    M = robot.getMassMatrix( q );
    M_inv = M \ eye( size( M ) );
    k = 0.01;
    Lambda_inv = J * M_inv * J' + ( k * k ) * eye( 6, 6 );
    Lambda = Lambda_inv \ eye( size( Lambda_inv ) );
    J_inv = M_inv * J' * Lambda;

    % New joint configuration
    dq_0 = J_inv * tw;
    del_q_0 = dq_0 * dt;
    q = q + del_q_0;

    % Store new joint configuration
    q_record_shift( :, i ) = q;

    % Update
    q_last = q;
    p_last = p;
    R_last = R;

    robot.updateKinematics( q_record( :, i ) );
    robot2.updateKinematics( q );
    anim.update( t );

end

anim.close( );

%% Can Run Imitation Learning, but after checking the code works

%IMIT_LEARN_LATER

% Apply Savitzky-Golay Filter to smooth the joint data
polynomialOrder = 3; % Typically a low-order polynomial
frameSize = 11; % Must be an odd number, adjust based on your noise level

% Smooth each joint trajectory
q_record_smoothed = sgolayfilt(q_record, polynomialOrder, frameSize, [], 2);
q_record_shift_smoothed = sgolayfilt(q_record_shift, polynomialOrder, frameSize, [], 2);

% Plot the original smoothed data for comparison (optional)
figure;
plot(t_record, q_record_smoothed);
title('Smoothed Original Joint Trajectories');
xlabel('Time (s)');
ylabel('Joint Angles (rad)');

% Plot the shifted smoothed data for comparison (optional)
figure;
plot(t_record, q_record_shift_smoothed);
title('Smoothed Shifted Joint Trajectories');
xlabel('Time (s)');
ylabel('Joint Angles (rad)');

%% Save smoothed data as csv for replay
csvwrite('q_recorded_smoothed.csv', q_record_smoothed);
csvwrite('q_record_shift_smoothed.csv', q_record_shift_smoothed);

%IMIT_LEARN_LATER_END

%% Save raw data as csv for comparison
csvwrite('q_recorded.csv', q_record);
csvwrite('q_record_shift.csv', q_record_shift);

