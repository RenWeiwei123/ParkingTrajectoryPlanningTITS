function LoadCase(case_id)
global params_
load(['Case', num2str(case_id), '.mat']);
params_.task.x0 = vehicle_TPBV_.x0;
params_.task.y0 = vehicle_TPBV_.y0;
params_.task.theta0 = vehicle_TPBV_.theta0;
params_.task.xtf = vehicle_TPBV_.xtf;
params_.task.ytf = vehicle_TPBV_.ytf;
params_.task.thetatf = vehicle_TPBV_.thetatf;
params_.obstacle.num_obs = length(obstacle_vertexes_);
params_.obstacle.obs = obstacle_vertexes_;
params_.dilated_map = CreateDilatedCostmap();
WriteBoundaryValues();
end

function costmap = CreateDilatedCostmap()
global params_
xmin = params_.demo.xmin;
ymin = params_.demo.ymin;
resolution_x = params_.hybrid_astar.resolution_dx;
resolution_y = params_.hybrid_astar.resolution_dy;
costmap = zeros(params_.hybrid_astar.num_nodes_x, params_.hybrid_astar.num_nodes_y);

for ii = 1 : size(params_.obstacle.obs, 2)
    vx = params_.obstacle.obs{ii}.x;
    vy = params_.obstacle.obs{ii}.y;
    x_lb = min(vx);
    x_ub = max(vx);
    y_lb = min(vy);
    y_ub = max(vy);
    [Nmin_x, Nmin_y] = ConvertXYToIndex(x_lb,y_lb);
    [Nmax_x, Nmax_y] = ConvertXYToIndex(x_ub,y_ub);
    for jj = Nmin_x : Nmax_x
        for kk = Nmin_y : Nmax_y
            if (costmap(jj,kk) == 1)
                continue;
            end
            cur_x = xmin + (jj - 1) * resolution_x;
            cur_y = ymin + (kk - 1) * resolution_y;
            if (inpolygon(cur_x, cur_y, params_.obstacle.obs{ii}.x, params_.obstacle.obs{ii}.y) == 1)
                costmap(jj,kk) = 1;
            end
        end
    end
end
length_unit = 0.5 * (resolution_x + resolution_y);
basic_elem = strel('disk', ceil(params_.vehicle.radius / length_unit));
costmap = imdilate(costmap, basic_elem);
end

function [ind1,ind2] = ConvertXYToIndex(x,y)
global params_
ind1 = ceil((x - params_.demo.xmin) / params_.hybrid_astar.resolution_dx) + 1;
ind2 = ceil((y - params_.demo.ymin) / params_.hybrid_astar.resolution_dy) + 1;
if ((ind1 <= params_.hybrid_astar.num_nodes_x)&&(ind1 >= 1)&&(ind2 <= params_.hybrid_astar.num_nodes_y)&&(ind2 >= 1))
    return;
end
if (ind1 > params_.hybrid_astar.num_nodes_x)
    ind1 = params_.hybrid_astar.num_nodes_x;
elseif (ind1 < 1)
    ind1 = 1;
end
if (ind2 > params_.hybrid_astar.num_nodes_y)
    ind2 = params_.hybrid_astar.num_nodes_y;
elseif (ind2 < 1)
    ind2 = 1;
end
end

function WriteBoundaryValues()
global params_
delete('BV');
fid = fopen('BV', 'w');
fprintf(fid, '1  %f\r\n', params_.task.x0);
fprintf(fid, '2  %f\r\n', params_.task.y0);
fprintf(fid, '3  %f\r\n', params_.task.theta0);
fprintf(fid, '4  %f\r\n', params_.task.xtf);
fprintf(fid, '5  %f\r\n', params_.task.ytf);
fprintf(fid, '6  %f\r\n', params_.task.thetatf);
fclose(fid);
end