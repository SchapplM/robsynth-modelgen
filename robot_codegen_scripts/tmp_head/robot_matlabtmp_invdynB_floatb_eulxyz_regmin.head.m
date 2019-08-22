% Calculate base parameter regressor for vector of inverse dynamics generalized base forces for
% %RN%
% Use Code from Maple symbolic Code Generation
%
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_QJDD%
% %INPUT_PHIB%
% %INPUT_XDB%
% %INPUT_XDDB%
% g [3x1]
%   gravitation vector in world frame [m/s^2]
% %INPUT_PKIN%
%
% Output:
% tauB_reg [6x%NMPVFLOATB%]
%   base parameter regressor for generalized base forces of inverse dynamics
%   (contains inertial, gravitational coriolis and centrifugal forces)
%   base moment as Euler angle moment

% %VERSIONINFO%

function tauB_reg = %FN%(qJ, qJD, qJDD, phi_base, xD_base, xDD_base, g, pkin)
