% Calculate minimal parameter regressor of joint inertia matrix time derivative for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% qD [%NQJ%x1]
%   Generalized velocities (joint velocities) [rad/s]
% pkin [%NKP%x1]
%   kinematic parameters
% 
% Output:
% MMD_reg [((%NQJ%+1)*%NQJ%/2)x%NMPV%]
%   minimal parameter regressor of inerta matrix time derivative
%   (only lower left triangular matrix (including diagonal) due to symmetry

% %VERSIONINFO%

function MMD_reg = %FN%(q, qD, ...
  pkin)
