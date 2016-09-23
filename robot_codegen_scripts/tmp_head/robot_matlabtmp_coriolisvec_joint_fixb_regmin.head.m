% Calculate minimal parameter regressor of coriolis joint torque vector for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% qD [%NQJ%x1]
%   Generalized velocities (joint velocities) [rad/s]
% a_mdh, d_mdh, q_offset_mdh [%NJ%x1]
%   kinematic parameters
% 
% Output:
% taug_reg [%NJ%x%NMPV%]
%   minimal parameter regressor of coriolis joint torque vector

% %VERSIONINFO%

function tauc_reg = %FN%(q, qD, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh%KCPARG%)
