% Calculate minimal parameter regressor of Gravitation load for parallel robot
% %RN%
% Use Code from Maple symbolic Code Generation
%
% Input:
% %INPUT_XP%
% %INPUT_QJ_P%
% %INPUT_LEGFRAME%
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% %INPUT_PKIN%
% %INPUT_KOPPEL%

% Output:
% tau_reg [%N_XP%x%NMPVPARA%]
%   minimal parameter regressor of Gravitation load for parallel robot
%   in task space

% %VERSIONINFO%

function tau_reg = %FN%(xP, qJ, g, legFrame, ...
  koppelP, pkin)
