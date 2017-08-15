% Calculate inertial parameters regressor of floating base kinetic energy for
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
% T_reg [1x%NMPVFLOATB%]
%   minimal parameter regressor of kinetic energy

% %VERSIONINFO%

function T_reg = %FN%(q, qD, phi_base, xD_base, pkin)
