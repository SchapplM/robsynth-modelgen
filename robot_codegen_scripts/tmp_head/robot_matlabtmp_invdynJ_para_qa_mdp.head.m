% Calculate minimal parameter regressor of inverse dynamics forces for
% %RN%
% Use Code from Maple symbolic Code Generation
%
% Input:
% %INPUT_XP%
% %INPUT_XDP%
% %INPUT_XDDP%
% %INPUT_QJ_P%
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% %INPUT_LEGFRAME%
% %INPUT_PKIN%
% %INPUT_KOPPEL%
% %INPUT_MDPFIXB_P%

% Output:
% tauA [%N_XP%x1]
%   inverse dynamics force vector in actuation coordinates

% %VERSIONINFO%

function tauA = %FN%(xP, xDP, xDDP, qJ, g, legFrame, ...
  koppelP, pkin, MDP)
