% Calculate inertia matrix for parallel robot
% %RN%
% Use Code from Maple symbolic Code Generation
%
% Input:
% %INPUT_XP%
% %INPUT_QJ_P%
% %INPUT_LEGFRAME%
% %INPUT_KOPPEL%
% %INPUT_PKIN%
% %INPUT_M_P%
% %INPUT_MR_P%
% %INPUT_IF_P%
%
% Output:
% MA [%N_LEGS%x%N_LEGS%]
%   inertia matrix in actuated joint coordinates

% %VERSIONINFO%

function MA = %FN%(xP, qJ, legFrame, ...
  koppelP, pkin, m, mrSges, Ifges)
