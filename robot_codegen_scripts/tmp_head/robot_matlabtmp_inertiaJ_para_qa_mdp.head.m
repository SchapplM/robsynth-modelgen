% Calculate minimal parameter regressor of inertia matrix for parallel robot
% %RN%
% Use Code from Maple symbolic Code Generation
%
% Input:
% %INPUT_XP%
% %INPUT_QJ_P%
% %INPUT_LEGFRAME%
% %INPUT_PKIN%
% %INPUT_KOPPEL%
% %INPUT_MDPFIXB_P%

% Output:
% MMA [%N_XP%x%N_XP%]
%   minimal parameter regressor of inertia matrix for parallel robot
%   in actuation coordinates

% %VERSIONINFO%

function MMA = %FN%(xP, qJ, legFrame, ...
  koppelP, pkin, MDP)
