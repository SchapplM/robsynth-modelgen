% Calculate matrix of centrifugal and coriolis load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_XP%
% %INPUT_XPD%
% %INPUT_QJ_P%
% %INPUT_LEGFRAME%
% %INPUT_PKIN%
% %INPUT_M_P%
% %INPUT_MR_P%
% %INPUT_RSP%
% %INPUT_IF_P%
% %INPUT_KOPPEL%
% 
% Output:
% Cq [%NQJ%x%NQJ%]
%   matrix of coriolis and centrifugal joint torques

% %VERSIONINFO%

function Cq = %FN%(xP, xPD, qJ, legFrame, ...
  koppelP, rSP, pkin, m, mrSges, Ifges)
