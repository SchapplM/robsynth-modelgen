% Calculate inertial parameters regressor of inverse dynamics base forces vector with Newton-Euler for
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
% tauB_reg [6x(%NQJ%*10)]
%   inertial parameter regressor of inverse dynamics base forces vector

% %VERSIONINFO%

function tauB_reg = %FN%(qJ, qJD, qJDD, g, pkin)
