% Calculate inertial parameters regressor of inverse dynamics with ic joint torque vector for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_QJDD%
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% %INPUT_PKIN%
% 
% Output:
% tau_reg [%NAJ%x(%NQJ%*10)]
%   inertial parameter regressor of inverse dynamics with ic joint torque vector

% %VERSIONINFO%

function tau_reg = %FN%(qJ, qJD, qJDD, g, pkin)
