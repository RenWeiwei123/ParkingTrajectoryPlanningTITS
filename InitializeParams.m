function InitializeParams()
global params_
params_.user.enable_video_recorder = 0;
params_.user.hybrid_astar_max_iter = 500;
params_.user.optimization_max_iter = 10;
params_.user.enable_stc_box_demo = 0;

params_.demo.xmin = -20;
params_.demo.xmax = 20;
params_.demo.ymin = -20;
params_.demo.ymax = 20;
params_.demo.colorpool = [237,28,36; 0,162,232; 255,127,39; 218,112,214; 255,192,203; 123,104,238;0,0,255;0,0,139;119,136,153;30,144,255;70,130,180;0,191,255;0,139,139;255,102,0;0,250,154;127,255,0;154,205,50;255,215,0;205,133,63;128,0,0;0,255,255;240,128,128;255,0,0;105,105,105;169,169,169;192,192,192;0,0,0] ./ 255;

params_.vehicle.lw = 2.8; % wheelbase
params_.vehicle.lf = 0.96; % front hang length
params_.vehicle.lr = 0.929; % rear hang length
params_.vehicle.lb = 1.942; % width
params_.vehicle.length = params_.vehicle.lw + params_.vehicle.lf + params_.vehicle.lr;
params_.vehicle.hypotenuse_length = hypot(params_.vehicle.length, params_.vehicle.lb);
params_.vehicle.radius = hypot(0.25 * params_.vehicle.length, 0.5 * params_.vehicle.lb); % Dual disk radius
params_.vehicle.r2p = 0.25 * params_.vehicle.length - params_.vehicle.lr;
params_.vehicle.f2p = 0.75 * params_.vehicle.length - params_.vehicle.lr;
params_.vehicle.vmax = 4.0;
params_.vehicle.amax = 4.0;
params_.vehicle.phymax = 0.85;
params_.vehicle.wmax = 1.0;
params_.vehicle.kappa_max = tan(params_.vehicle.phymax) / params_.vehicle.lw;
params_.vehicle.turning_radius_min = abs(1.0 / params_.vehicle.kappa_max);
params_.vehicle.threshold_s = (params_.vehicle.vmax^2) / params_.vehicle.amax;

params_.hybrid_astar.resolution_dx = 0.4;
params_.hybrid_astar.resolution_dy = 0.4;
params_.hybrid_astar.resolution_dtheta = 0.2;
params_.hybrid_astar.num_nodes_x = ceil((params_.demo.xmax - params_.demo.xmin) / params_.hybrid_astar.resolution_dx) + 1;
params_.hybrid_astar.num_nodes_y = ceil((params_.demo.ymax - params_.demo.ymin) / params_.hybrid_astar.resolution_dy) + 1;
params_.hybrid_astar.num_nodes_theta = ceil(2 * pi / params_.hybrid_astar.resolution_dtheta) + 1;
params_.hybrid_astar.penalty_for_backward = 1.0;
params_.hybrid_astar.penalty_for_direction_change = 3.0;
params_.hybrid_astar.penalty_for_steering_change = 0.001;
params_.hybrid_astar.multiplier_H = 5.0;
params_.hybrid_astar.multiplier_H_for_2dim_A_star = 3.0;
params_.hybrid_astar.simulation_step = 0.7;

params_.opti.nfe = 100;
params_.dt_for_resampling = 0.001;
params_.opti.stc.ds = 0.1;
params_.opti.stc.smax = 5.0;
params_.opti.acc_tol = 0.000001;
params_.opti.cost_function_weight = 10000;
params_.opti.trust_region_ds = 2;
params_.opti.trust_region_dtheta = 1;
params_.opti.incremental_tf = 3.0;
end