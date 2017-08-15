% Zeitableitung der rotatorischen Teilmatrix der analytischen Jacobi-Matrix für beliebiges Segment von
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
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt.
%   Wie in %RN%_fkine_fixb_rotmat_mdh_sym_varpar.m (1=Basis).
% %INPUT_PKIN%
% 
% Output:
% JaD_rot [3x%NQJ%]
%   Zeitableitung der rotatorischen Teilmatrix der analytischen Jacobi-Matrix

% %VERSIONINFO%

function JaD_rot = %FN%(q, qD, link_index, ...
  pkin)
