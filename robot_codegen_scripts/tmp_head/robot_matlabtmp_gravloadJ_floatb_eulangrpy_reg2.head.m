% Calculate inertial parameter regressor for Gravitation load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% g [3x1]
%   gravitation vector in world frame [m/s^2]
% pkin [%NKP%x1]
%   kinematic parameters
% 
% Output:
% taug_reg [%NQJ%x(10*%NL%)]
%   inertial parameter regressor for joint torques required to compensate gravitation load

% %VERSIONINFO%

function taug = %FN%(q, phi_base, g, pkin)
