% Calculate base parameter regressor of vector of centrifugal and coriolis load on the base for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_PHIB%
% %INPUT_RB%
% %INPUT_XDB%
% %INPUT_PKIN%
% 
% Output:
% tauc_reg [6x%NMPVFLOATB%]
%   base parameter regressor for generalized base forces and torques 
%   required to compensate coriolis and centrifugal load

% %VERSIONINFO%

function tauc_reg = %FN%(qJ, qJD, phi_base, xD_base, pkin)
