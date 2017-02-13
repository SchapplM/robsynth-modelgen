% Calculate inertial parameters regressor of potential energy for
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
% U_reg [1x(%NL%*10)]
%   inertial parameter regressor of Potential energy

% %VERSIONINFO%

function U_reg = %FN%(q, r_base, phi_base, g, pkin)
