% Calculate minimal parameter regressor of gravitation load for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% g_base [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% %INPUT_PKIN%
% 
% Output:
% taug_reg [%NQJ%x%NMPVFIXB%]
%   minimal parameter regressor of gravitation joint torque vector

% %VERSIONINFO%

function taug_reg = %FN%(q, g, ...
  pkin)
