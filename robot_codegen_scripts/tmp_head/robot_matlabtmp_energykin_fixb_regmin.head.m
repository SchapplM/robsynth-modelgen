% Calculate minimal parameter regressor of fixed base kinetic energy for
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
% T_reg [1x%NMPV%]
%   minimal parameter regressor of kinetic energy

% %VERSIONINFO%

function T_reg = %FN%(q, qD, pkin)
