% Rotatorische Teilmatrix der geometrischen Jacobi-Matrix für beliebiges Segment von
% %RN%
% Use Code from Maple symbolic Code Generation
%
% Geometrische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorgeschwindigkeit und Geschw. der verallgemeinerten Koordinaten.
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
% Jg_rot [3x%NQJ%]
%   Rotatorische Teilmatrix der geometrischen Jacobi-Matrix

% %VERSIONINFO%

function Jg_rot = %FN%(qJ, link_index, pkin)
