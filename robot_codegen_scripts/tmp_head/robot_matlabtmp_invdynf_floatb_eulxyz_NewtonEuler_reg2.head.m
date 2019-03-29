% Calculate inertial parameter regressor for vector of inverse dynamics cutting forces with Newton-Euler for
% %RN%
% Use Code from Maple symbolic Code Generation
%
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_QJDD%
% %INPUT_RB%
% %INPUT_PHIB%
% %INPUT_XDB%
% %INPUT_XDDB%
% g [3x1]
%   gravitation vector in world frame [m/s^2]
% %INPUT_PKIN%
%
% Output:
% f_new_reg [(3*%NL%)x(10*%NL%)]
%   inertial parameter regressor for cutting forces of inverse dynamics
%   (contains inertial, gravitational coriolis and centrifugal forces)

% %VERSIONINFO%

function f_new_reg = %FN%(qJ, qJD, qJDD, phi_base, xD_base, xDD_base, g, pkin)
