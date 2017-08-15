% Calculate inertial parameters regressor of potential energy for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_RB%
% %INPUT_PHIB%
% g [3x1]
%   gravitation vector in world frame [m/s^2]
% %INPUT_PKIN%
% 
% Output:
% U_reg [1x%NMPVFLOATB%]
%   minimal parameter regressor of Potential energy

% %VERSIONINFO%

function U_reg = %FN%(q, r_base, phi_base, g, pkin)
