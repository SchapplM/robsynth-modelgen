% Calculate minimal parameter regressor of coriolis joint torque vector for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Joint angles [rad]
% qD [%NQJ%x1]
%   Joint velocities [rad/s]
% qD [%NQJ%x1]
%   Joint accelerations [rad/s]
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [%NKP%x1]
%   kinematic parameters
% 
% Output:
% tau_reg [%NQJ%x%NMPV%]
%   minimal parameter regressor of inverse dynamics joint torque vector

% %VERSIONINFO%

function tau_reg = %FN%(q, qD, qDD, g, ...
  pkin)
