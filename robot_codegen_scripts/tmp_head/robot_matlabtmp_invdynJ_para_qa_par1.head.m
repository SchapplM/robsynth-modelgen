% Calculate vector of inverse dynamics forces for parallel robot
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
% %INPUT_KOPPEL%
% %INPUT_PKIN%
% %INPUT_M_P%
% %INPUT_R_P%
% %INPUT_IC_P%
%
% Output:
% tauA [%N_LEGS%x1]
%   forces of inverse dynamics (contains inertial, gravitational coriolis and centrifugal forces)
%   in actuated joint coordinates

% %VERSIONINFO%

function tauA = %FN%(xP, xDP, xDDP, qJ, g, legFrame, ...
  koppelP, pkin, m, rSges, Icges)
