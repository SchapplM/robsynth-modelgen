% Calculate inertial parameters regressor of inverse dynamics cutting forces vector with Newton-Euler for
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
% f_new_reg [(3*%NL%)x(%NL%*10)]
%   inertial parameter regressor of inverse dynamics cutting forces vector

% %VERSIONINFO%

function f_new_reg = %FN%(qJ, qJD, qJDD, g, pkin)
