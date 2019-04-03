% Calculate minimal parameter regressor of vector of centrifugal and coriolis load for parallel robot
% %RN%
% Use Code from Maple symbolic Code Generation
%
% Input:
% %INPUT_XP%
% %INPUT_XDP%
% %INPUT_QJ_P%
% %INPUT_LEGFRAME%
% %INPUT_PKIN%
% %INPUT_KOPPEL%
% %INPUT_MDPFIXB_P%

% Output:
% taucX [%N_XP%x1]
%   minimal parameter regressor of vector of coriolis and centrifugal joint torques
%   in task space

% %VERSIONINFO%

function taucX = %FN%(xP, xDP, qJ, legFrame, ...
  koppelP, pkin, MDP)
