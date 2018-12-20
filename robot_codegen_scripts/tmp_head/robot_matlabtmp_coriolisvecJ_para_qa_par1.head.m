% Calculate vector of centrifugal and coriolis load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
%
% Input:
% %INPUT_XP%
% %INPUT_XDP%
% %INPUT_QJ_P%
% %INPUT_LEGFRAME%
% %INPUT_KOPPEL%
% %INPUT_PKIN%
% %INPUT_M_P%
% %INPUT_R_P%
% %INPUT_IC_P%
%
% Output:
% tauc [%N_LEGS%x1]
%   forces required to compensate Coriolis and centrifugal joint torques
%   in actuated joint coordinates

% %VERSIONINFO%

function tauc = %FN%(xP, xDP, qJ, legFrame, ...
  koppelP, pkin, m, rSges, Icges)
