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
% %INPUT_MDPFIXB_P%

% Output:
% taugA [%N_XP%x1]
%   minimal parameter regressor of Gravitation load for parallel robot
%   in actuation coordinates

% %VERSIONINFO%

function taugA = %FN%(xP, qJ, g, legFrame, ...
  koppelP, pkin, MDP)
