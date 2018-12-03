% Analytische Jacobi-Matrix für
% Use Code from Maple symbolic Code Generation
% 
% analytische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und aktiven Koordinaten.
% Zeitableitung der Winkeldarstellung des Endeffektors in Basis-Koordinaten
% 
% Input:
% %INPUT_XP%
% %INPUT_QJ_P%
% %INPUT_LEGFRAME%
% %INPUT_PKIN%
% %INPUT_KOPPEL%

% Output:
% Jinv [%N_XP%x%N_XP%]
%   Analytische Jacobi-Matrix

% %VERSIONINFO%

function Jinv = %FN%(xP, qJ, pkin, koppelP, ...
legFrame)


