% Geometrische Jacobi-Matrix für beliebiges Segment von
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Geometrische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorgeschwindigkeit und Geschw. der verallgemeinerten Koordinaten.
% 
% Input:
% %INPUT_QJ%
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt. (0=Basis).
%   Siehe auch: bsp_3T1R_fkine_fixb_rotmat_mdh_sym_varpar.m
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% %INPUT_PKIN%
% 
% Output:
% Jg [6x%NQJ%]
%   Geometrische Jacobi-Matrix

% %VERSIONINFO%

function Jg = %FN%(qJ, link_index, r_i_i_C, pkin)
