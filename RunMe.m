% ==============================================================================
%  MATLAB Source Codes for "Optimization-based trajectory planning for
%  autonomous parking with irregularly placed obstacles: A lightweight
%  iterative framework".
% ==============================================================================
%   Copyright (C) 2022 Bai Li
%   Users should cite the following article when utilizing these source codes.
%   Bai Li, Tankut Acarman, Youmin Zhang, et al., “Optimization-based
%   trajectory planning for autonomous parking with irregularly placed
%   obstacles: A lightweight iterative framework,” IEEE Transactions on
%   Intelligent Transportation Systems, accepted on Aug. 27, 2021.
% ==============================================================================
%   2021.12.16
% ==============================================================================
clear all; close all; clc;
global params_
for case_id = 1 : 15
    InitializeParams();
    LoadCase(case_id);
    [params_.ha_x, params_.ha_y, params_.ha_theta] = SearchViaFTHA();
    asd(case_id);
    is_good = OptiMethod();
    if (is_good)
        plot(params_.opti_x, params_.opti_y, 'b', 'LineWidth', 3);
    end
end