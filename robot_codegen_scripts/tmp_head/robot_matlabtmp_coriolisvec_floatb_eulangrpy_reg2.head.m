% Calculate inertial parameter regressor of vector of centrifugal and coriolis load for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% qD [%NQJ%x1]
%   Generalized velocities (joint velocities) [rad/s]
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% xD_base [6x1]
%   time derivative of 
%   r_base (3x1 Base position in world frame) and 
%   phi_base (3x1)
% pkin [%NKP%x1]
%   kinematic parameters
% 
% Output:
% tauc_reg [(6+%NQJ%)x(%NL%*10)]
%   inertial parameter regressor for generalized forces and torques 
%   required to compensate coriolis and centrifugal load (floating base)

% %VERSIONINFO%

function tauc_reg = %FN%(q, qD, phi_base, xD_base, pkin)
