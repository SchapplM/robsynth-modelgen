% Calculate inertial parameters regressor of potential energy for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% %INPUT_PKIN%
% 
% Output:
% U_reg [1x(%NQJ%*10)]
%   inertial parameter regressor of Potential energy

% %VERSIONINFO%

function U_reg = %FN%(q, g, pkin)
