% Zeitableitung der Rotationsmatrix-Jacobi-Matrix für beliebiges Segment von
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Rotationsmatrix-Jacobi-Matrix: Differentieller Zusammenhang zwischen
% gestapelter Endeffektor-Rotationsmatrix und verallgemeinerten Koordinaten.
% Zeitableitung: Die Gradientenmatrix wird nochmal nach der Zeit abgeleitet.
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt. (0=Basis).
%   Siehe auch: %RN%_fkine_fixb_rotmat_mdh_sym_varpar.m
% %INPUT_PKIN%
% 
% Output:
% JRD_rot [9x%NQJ%]
%   Zeitableitung der Jacobi-Matrix der Endeffektor-Rotationsmatrix

% %VERSIONINFO%

function JRD_rot = %FN%(qJ, qJD, link_index, pkin)
