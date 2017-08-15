% Zeitableitung der geometrischen Jacobi-Matrix für beliebiges Segment von
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% geometrische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
% 
% Input:
% %INPUT_Q%
% %INPUT_QD%
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt.
%   Wie in %RN%_fkine_fixb_rotmat_mdh_sym_varpar.m (1=Basis).
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% %INPUT_PKIN%
% 
% Output:
% Jg [6x%NQJ%]
%   Zeitableitung der geometrischen Jacobi-Matrix

% %VERSIONINFO%

function JgD = %FN%(qJ, qJD, link_index, r_i_i_C, ...
  pkin)



