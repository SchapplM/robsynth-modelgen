% Calculate base parameter regressor for vector of inverse dynamics joint torques for
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
% tau_reg [%NQJ%x%NMPVFLOATB%]
%   base parameter regressor for joint torques of inverse dynamics
%   (contains inertial, gravitational coriolis and centrifugal forces)

% %VERSIONINFO%

function tau_reg = %FN%(qJ, qJD, qJDD, phi_base, xD_base, xDD_base, g, pkin)
