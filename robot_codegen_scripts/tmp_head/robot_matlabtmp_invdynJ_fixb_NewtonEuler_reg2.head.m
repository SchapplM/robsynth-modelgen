% Calculate inertial parameters regressor of inverse dynamics joint torque vector with Newton-Euler for
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
% tauJ_reg [%NQJ%x(%NQJ%*10)]
%   inertial parameter regressor of inverse dynamics joint torque vector

% %VERSIONINFO%

function tauJ_reg = %FN%(qJ, qJD, qJDD, g, pkin)
