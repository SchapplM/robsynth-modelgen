% Rotatorische Teilmatrix der Rotationsmatrix-Jacobi-Matrix für beliebiges Segment von
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Rotationsmatrix-Jacobi-Matrix: Differentieller Zusammenhang zwischen
% gestapelter Endeffektor-Rotationsmatrix und verallgemeinerten Koordinaten.
% 
% 
% Input:
% %INPUT_QJ%
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt. (0=Basis).
%   Siehe auch: %RN%_fkine_fixb_rotmat_mdh_sym_varpar.m
% %INPUT_PKIN%
% 
% Output:
% JR_rot [9x%NQJ%]
%   Jacobi-Matrix der Endeffektor-Rotationsmatrix

% %VERSIONINFO%

function JR_rot = %FN%(qJ, link_index, pkin)
