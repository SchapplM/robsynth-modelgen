% Calculate inertial parameter regressor for vector of inverse dynamics joint toques with Newton-Euler for
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
% tauJ_reg [6x(10*%NL%)]
%   inertial parameter regressor for joint toques of inverse dynamics
%   (contains inertial, gravitational coriolis and centrifugal forces)
%   base moment as Euler angle moment

% %VERSIONINFO%

function tauJ_reg = %FN%(qJ, qJD, qJDD, phi_base, xD_base, xDD_base, g, pkin)
