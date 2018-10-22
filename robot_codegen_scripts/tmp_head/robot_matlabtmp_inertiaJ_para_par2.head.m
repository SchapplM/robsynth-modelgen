% Calculate joint inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_XP%
% %INPUT_QJ_P%
% %INPUT_LEGFRAME%
% %INPUT_PKIN%
% %INPUT_M_P%
% %INPUT_MR_P%
% %INPUT_IF_P%
% %INPUT_KOPPEL%
% 
% Output:
% Mq [%NQJ%x%NQJ%]
%   inertia matrix

% %VERSIONINFO%

function Mq = %FN%(xP, qJ, legFrame, ...
  koppelP, pkin, m, mrSges, Ifges)
