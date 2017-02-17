% Calculate minimal parameter regressor of joint-base floating base inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% pkin [%NKP%x1]
%   kinematic parameters
% 
% Output:
% MM_reg [(6*%NQJ%)x%NMPVFIXB%]
%   minimal parameter regressor of joint-base floating base inertia matrix

% %VERSIONINFO%

function MM_reg = %FN%(q, phi_base, pkin)
