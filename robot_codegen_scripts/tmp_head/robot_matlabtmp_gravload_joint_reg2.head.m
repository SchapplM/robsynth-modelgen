% Calculate inertial parameters regressor of gravitation load for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% g_base [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [%NKP%x1]
%   kinematic parameters
% 
% Output:
% taug_reg [%NQJ%x(%NQJ%*10)]
%   inertial parameter regressor of gravitation joint torque vector

% %VERSIONINFO%

function taug_reg = %FN%(q, g, pkin)
