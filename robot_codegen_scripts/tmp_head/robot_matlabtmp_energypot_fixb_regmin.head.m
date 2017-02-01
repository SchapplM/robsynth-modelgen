% Calculate minimal parameter regressor of potential energy for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) [rad]
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% pkin [%NKP%x1]
%   kinematic parameters
% 
% Output:
% U_reg [1x%NMPV%]
%   minimal parameter regressor of Potential energy

% %VERSIONINFO%

function U_reg = %FN%(q, g, ...
  pkin)
