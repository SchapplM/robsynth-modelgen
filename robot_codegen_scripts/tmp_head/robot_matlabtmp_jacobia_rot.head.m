% Rotatorische Teilmatrix der analytischen Jacobi-Matrix für Segment Nr. %LIJAC% (1=Basis) von
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
% %INPUT_QJ%
% %INPUT_PKIN%
% 
% Output:
% Ja_rot [3x%NQJ%]
%   Rotatorische Teilmatrix der analytischen Jacobi-Matrix

% %VERSIONINFO%

function Ja_rot = %FN%(qJ, ...
  pkin)
