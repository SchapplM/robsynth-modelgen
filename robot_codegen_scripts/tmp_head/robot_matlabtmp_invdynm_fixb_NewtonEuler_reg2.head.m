% Calculate inertial parameters regressor of inverse dynamics cutting torque vector with Newton-Euler for
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
% m_new_reg [(3*%NL%)x(%Nl%*10)]
%   inertial parameter regressor of inverse dynamics cutting torque vector

% %VERSIONINFO%

function m_new_reg = %FN%(qJ, qJD, qJDD, g, pkin)
