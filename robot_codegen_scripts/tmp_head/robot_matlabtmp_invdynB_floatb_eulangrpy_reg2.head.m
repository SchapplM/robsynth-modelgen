% Calculate inertial parameter regressor for vector of inverse dynamics generalized base forces for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_QD%
% %INPUT_QDD%
% %INPUT_RB%
% %INPUT_PHIB%
% %INPUT_XDB%
% %INPUT_XDDB%
% g [3x1]
%   gravitation vector in world frame [m/s^2]
% %INPUT_PKIN%
% 
% Output:
% tauB_reg [6x(10*%NL%)]
%   inertial parameter regressor for generalized base forces of inverse dynamics
%   (contains inertial, gravitational coriolis and centrifugal forces)
%   base moment as Euler angle moment

% %VERSIONINFO%

function tauB_reg = %FN%(q, qD, qDD, phi_base, xD_base, xDD_base, g, pkin)
