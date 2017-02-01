% Calculate inertial parameters regressor of coriolis joint torque vector for
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
% taug_reg [%NQJ%x(%NQJ%*10)]
%   minimal parameter regressor of coriolis joint torque vector

% %VERSIONINFO%

function tauc_reg = %FN%(q, qD, ...
  pkin)
