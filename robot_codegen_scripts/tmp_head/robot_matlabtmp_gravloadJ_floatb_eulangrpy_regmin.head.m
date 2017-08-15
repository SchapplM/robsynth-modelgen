% Calculate base parameter regressor for Gravitation load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_PHIB%
% g [3x1]
%   gravitation vector in world frame [m/s^2]
% %INPUT_PKIN%
% 
% Output:
% taug_reg [%NQJ%x%NMPVFLOATB%]
%   base parameter regressor for joint torques required to compensate gravitation load

% %VERSIONINFO%

function taug = %FN%(q, phi_base, g, pkin)
