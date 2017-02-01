% Rotatorische Teilmatrix der geometrischen Jacobi-Matrix für beliebiges Segment von
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% geometrische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
% 
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) (generalized coordinates) [rad]
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt.
%   Wie in %RN%_fkine_fixb_rotmat_mdh_sym_varpar.m
% a_mdh, d_mdh, q_offset_mdh, ... [%NJ%x1]
%   kinematic parameters
% 
% Output:
% Jg_rot [3x%NQJ%]
%   Rotatorische Teilmatrix der geometrischen Jacobi-Matrix

% %VERSIONINFO%

function Jg_rot = %FN%(q, link_index, ...
  pkin)
