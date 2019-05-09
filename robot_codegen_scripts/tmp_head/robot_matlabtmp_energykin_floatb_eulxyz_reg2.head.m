% Calculate inertial parameters regressor of floating base kinetic energy for
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
% T_reg [1x(%NL%*10)]
%   inertial parameter regressor of kinetic energy

% %VERSIONINFO%

function T_reg = %FN%(qJ, qJD, phi_base, xD_base, pkin)
