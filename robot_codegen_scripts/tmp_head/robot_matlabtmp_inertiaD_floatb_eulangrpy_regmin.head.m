% Calculate base parameter regressor for inertia matrix time derivative for
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
% MD_reg [(((6+%NQJ%)+1)*(6+%NQJ%)/2)x%NMPVFLOATB%]
%   base parameter regressor for full time derivative of inertia matrix (for base and joint dynamics)
%   (only lower left triangular matrix (including diagonal) due to symmetry

% %VERSIONINFO%

function MD_reg = %FN%(q, qD, phi_base, xD_base, pkin)
