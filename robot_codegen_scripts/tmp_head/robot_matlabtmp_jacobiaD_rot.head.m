% Zeitableitung der rotatorischen Teilmatrix der analytischen Jacobi-Matrix für Segment Nr. %LIJAC% (1=Basis) von
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% analytische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
% Zeitableitung der Winkeldarstellung des Endeffektors in Basis-Koordinaten
% 
% Winkeldarstellung: RPY-Winkel, rotx(alpha)*roty(beta)*rotz(gamma)
% 
% Input:
% %INPUT_Q%
% %INPUT_QD%
% %INPUT_PKIN%
% 
% Output:
% JaD_rot [3x%NQJ%]
%   Zeitableitung der rotatorischen Teilmatrix der analytischen Jacobi-Matrix

% %VERSIONINFO%

function JaD_rot = %FN%(q, qD, ...
  pkin)
