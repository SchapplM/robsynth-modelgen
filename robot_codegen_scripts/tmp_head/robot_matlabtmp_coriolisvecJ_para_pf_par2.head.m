% Calculate vector of centrifugal and Coriolis load for parallel robot
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
% %INPUT_MR_P%
% %INPUT_IF_P%
%
% Output:
% taucX [%N_LEGS%x1]
%   forces required to compensate Coriolis and centrifugal joint torques
%   in platform coordinates

% %VERSIONINFO%

function taucX = %FN%(xP, xDP, qJ, legFrame, ...
  koppelP, pkin, m, mrSges, Ifges)
