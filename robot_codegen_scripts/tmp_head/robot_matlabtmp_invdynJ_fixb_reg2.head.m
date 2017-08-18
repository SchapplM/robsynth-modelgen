% Calculate inertial parameters regressor of inverse dynamics joint torque vector for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_QD%
% %INPUT_QDD%
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% %INPUT_PKIN%
% 
% Output:
% tau_reg [%NQJ%x(%NQJ%*10)]
%   inertial parameter regressor of inverse dynamics joint torque vector

% %VERSIONINFO%

function tau_reg = %FN%(qJ, qJD, qJDD, g, pkin)
