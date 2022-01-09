clear all
close all
rng(0);

N = 7;
confidence_level = 0.8;
SafetyRadius = 0.2; % should manually keep consistent with the initial value in ARobotarium
obs_robot_idx = [6 7];

% rng(99);
x_rand_span_x = 0.02*randi([3 4],1,N); % setting up position error range for each robot, rand_span serves as the upper bound of uncertainty for each of the robot
x_rand_span_y = 0.02*randi([1 4],1,N); %
v_rand_span = 0.005*ones(2,N); % setting up velocity error range for each robot
dxi = rand([2 6]);
x_observe = rand([3 6]);

si_barrier_certificate = create_si_pr_barrier_certificate_decentralized('SafetyRadius', 2*SafetyRadius, 'Confidence', confidence_level, 'obs_robot_idx', obs_robot_idx);%

dxi_r = si_barrier_certificate(dxi, x_observe(1:2, :), 'XRandSpan', [x_rand_span_x;x_rand_span_y],'URandSpan', v_rand_span);

