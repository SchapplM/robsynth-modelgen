% Calculate Gravitation load on the joints for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_XP%
% %INPUT_QJ_P%
% %INPUT_LEGFRAME%
% g [3x1]
%   gravitation vector in mdh base frame [m/s^2]
% %INPUT_PKIN%
% %INPUT_M_P%
% %INPUT_R_P%
% %INPUT_KOPPEL%
% 
% Output:
% taug [%N_LEGS%x1]
%   joint torques required to compensate gravitation load

% %VERSIONINFO%

function taug = %FN%(xP, qJ, g, legFrame, ...
  koppelP, pkin, m, rSges)
