% Calculate Gravitation load for parallel robot
% %RN%
% Use Code from Maple symbolic Code Generation
%
% Input:
% %INPUT_XP%
% %INPUT_QJ_P%
% %INPUT_LEGFRAME%
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% %INPUT_KOPPEL%
% %INPUT_PKIN%
% %INPUT_M_P%
% %INPUT_R_P%
%
% Output:
% taugA [%N_LEGS%x1]
%   forces required to compensate gravitation load
%   in actuated joint coordinates

% %VERSIONINFO%

function taugA = %FN%(xP, qJ, g, legFrame, ...
  koppelP, pkin, m, rSges)
