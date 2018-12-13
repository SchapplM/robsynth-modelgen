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
% %INPUT_MR_P%
% %INPUT_IF_P%
% 
% Output:
% c [%N_LEGS%x%N_LEGS%]
%   vector of coriolis and centrifugal joint torques
%   in actuated joint coordinates

% %VERSIONINFO%

function c = %FN%(xP, xDP, qJ, legFrame, ...
  koppelP, pkin, m, mrSges, Ifges)
