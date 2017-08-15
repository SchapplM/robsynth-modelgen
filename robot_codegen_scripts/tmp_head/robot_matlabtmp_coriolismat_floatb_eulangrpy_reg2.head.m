% Calculate inertial parameter regressor of matrix of centrifugal and coriolis load on the base and joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_QD%
% %INPUT_PHIB%
% %INPUT_RB%
% %INPUT_XDB%

% %INPUT_PKIN%
% 
% Output:
% cmat_reg [((6+%NQJ%)*(6+%NQJ%))x(%NL%*10)]
%   inertial parameter regressor of matrix of coriolis and centrifugal generalized forces
%   Gives coriolis base forces/torques and joint torques when multiplied with base and joint velocities

% %VERSIONINFO%

function Cq = %FN%(qJ, qJD, phi_base, xD_base, pkin)
