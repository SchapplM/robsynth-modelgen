% Calculate inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_XP%
% %INPUT_QJ_P%
% %INPUT_LEGFRAME%
% %INPUT_PKIN%
% %INPUT_M_P%
% %INPUT_R_P%
% %INPUT_IC_P%
% %INPUT_KOPPEL%
% 
% Output:
% M [%N_LEGS%x%N_LEGS%]
%   inertia matrix

% %VERSIONINFO%

function M = %FN%(xP, qJ, legFrame, ...
  koppelP, pkin, m, rSges, Icges)
