function [x, y, theta] = SearchAStarPath(start_config, end_config)
global params_
begin_config = start_config(1:2);
end_config = end_config(1:2);
grid_space_2D_ = cell(params_.hybrid_astar.num_nodes_x, params_.hybrid_astar.num_nodes_y);
init_node = zeros(1,11);
% Information of each element in each node:
% Dim # | Variable
%  1        x
%  2        y
%  3        f
%  4        g
%  5        h
%  6        is_in_openlist
%  7        is_in_closedlist
%  8-9      index of current node
%  10-11    index of parent node

init_node(1:2) = begin_config;
init_node(4) = 0;
init_node(5) = sum(abs(init_node(1:2) - end_config));
init_node(3) = init_node(4) + params_.hybrid_astar.multiplier_H_for_2dim_A_star * init_node(5) + 0.001 * randn;
init_node(6) = 1;
init_node(8:9) = Convert2DimConfigToIndex(begin_config);
init_node(10:11) = [-999,-999];
openlist_ = init_node;
goal_ind = Convert2DimConfigToIndex(end_config);
grid_space_2D_{init_node(8), init_node(9)} = init_node;
expansion_pattern = [-1 1; -1 0; -1 -1; 0 1; 0 -1; 1 1; 1 0; 1 -1] .* params_.hybrid_astar.resolution_dx;
expansion_length = [1.414; 1; 1.414; 1; 1; 1.414; 1; 1.414] .* params_.hybrid_astar.resolution_dx;
completeness_flag = 0;

iter = 0;
while ((~isempty(openlist_))&&(iter <= params_.hybrid_astar.num_nodes_x^2)&&(~completeness_flag))
    iter = iter + 1;
    % Locate the node with smallest f value in the openlist, and then name
    % it as cur_node and prepare for extension
    cur_node_order = find(openlist_(:,3) == min(openlist_(:,3))); cur_node_order = cur_node_order(end);
    cur_node = openlist_(cur_node_order, :);
    cur_config = cur_node(1:2);
    cur_ind = cur_node(8:9);
    cur_g = cur_node(4);
    % Remove cur_node from open list and add it in closed list
    openlist_(cur_node_order, :) = [];
    grid_space_2D_{cur_ind(1), cur_ind(2)}(6) = 0;
    grid_space_2D_{cur_ind(1), cur_ind(2)}(7) = 1;
    for ii = 1 : 8
        child_node_config = cur_config + expansion_pattern(ii,:);
        
        child_node_ind = Convert2DimConfigToIndex(child_node_config);
        child_g = cur_g + expansion_length(ii);
        child_h = sum(abs(child_node_config - end_config));
        child_f = child_g + params_.hybrid_astar.multiplier_H_for_2dim_A_star * child_h;
        child_node_prepare = [child_node_config, child_f, child_g, child_h, 1, 0, child_node_ind, cur_ind];
        % If the child node has been explored ever before
        if (~isempty(grid_space_2D_{child_node_ind(1), child_node_ind(2)}))
            % If the child has been within the closed list, abandon it and continue.
            if (grid_space_2D_{child_node_ind(1), child_node_ind(2)}(7) == 1)
                continue;
            end
            % The child must be in the open list now, then check if its
            % recorded parent deserves to be switched as our cur_node.
            if (grid_space_2D_{child_node_ind(1), child_node_ind(2)}(4) > child_g + 0.1)
                child_node_order1 = find(openlist_(:,8) == child_node_ind(1));
                child_node_order2 = find(openlist_(child_node_order1,9) == child_node_ind(2));
                openlist_(child_node_order1(child_node_order2), :) = [];
                grid_space_2D_{child_node_ind(1), child_node_ind(2)} = child_node_prepare;
                openlist_ = [openlist_; child_node_prepare];
            end
        else % Child node has never been explored before
            % If the child node is collison free
            if (Is2DNodeValid(child_node_config, child_node_ind))
                % If the child node is close to the goal point, then exit
                % directly because we only need the length value rather than the path.
                openlist_ = [openlist_; child_node_prepare];
                grid_space_2D_{child_node_ind(1), child_node_ind(2)} = child_node_prepare;
                if (sum(abs(child_node_ind - goal_ind)) == 0)
                    completeness_flag = 1;
                end
            else % If the child node involves collisons
                child_node_prepare(7) = 1;
                child_node_prepare(6) = 0;
                grid_space_2D_{child_node_ind(1), child_node_ind(2)} = child_node_prepare;
            end
        end
    end
end

if (completeness_flag)
    ind_vec = goal_ind;
    parent_ind = grid_space_2D_{goal_ind(1), goal_ind(2)}(10:11);
    while (parent_ind(1) > -1)
        ind_vec = [parent_ind; ind_vec];
        parent_ind = grid_space_2D_{parent_ind(1), parent_ind(2)}(10:11);
    end
    
    x = ((ind_vec(:,1) - 1) * params_.hybrid_astar.resolution_dx + params_.demo.xmin)';
    y = ((ind_vec(:,2) - 1) * params_.hybrid_astar.resolution_dy + params_.demo.ymin)';
    theta = x;
    theta(1) = start_config(3);
    theta(2) = start_config(3);
    for ii = 3 : size(x,2)
        theta(ii) = atan2(y(ii) - y(ii-1), x(ii) - x(ii-1));
        while (theta(ii) - theta(ii-2) < -0.5 * pi)
            theta(ii) = theta(ii) + pi;
        end
        while (theta(ii) - theta(ii-2) > 0.5 * pi)
            theta(ii) = theta(ii) - pi;
        end
    end
else
    x = [];
    y = [];
    theta = [];
end
end

function is_collision_free = Is2DNodeValid(child_node_config, child_node_ind)
is_collision_free = 1;
global params_
if (params_.dilated_map(child_node_ind(1), child_node_ind(2)) == 1)
    is_collision_free = 0;
    return;
end
if ((child_node_config(1) > params_.demo.xmax) || (child_node_config(1) < params_.demo.xmin) || (child_node_config(2) > params_.demo.ymax) || (child_node_config(2) < params_.demo.ymin))
    is_collision_free = 0;
    return;
end
end

function idx = Convert2DimConfigToIndex(config)
global params_
ind1 = ceil((config(1) - params_.demo.xmin) / params_.hybrid_astar.resolution_dx) + 1;
ind2 = ceil((config(2) - params_.demo.ymin) / params_.hybrid_astar.resolution_dy) + 1;
idx = [ind1, ind2];
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
idx = [ind1, ind2];
end