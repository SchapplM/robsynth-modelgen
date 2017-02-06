% Rotatorische Teilmatrix der analytischen Jacobi-Matrix für beliebiges Segment von
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
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) (generalized coordinates) [rad]
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt.
%   Wie in %RN%_fkine_fixb_rotmat_mdh_sym_varpar.m
% a_mdh, d_mdh, q_offset_mdh, ... [%NJ%x1]
%   kinematic parameters
% 
% Output:
% Ja_rot [3x%NQJ%]
%   Rotatorische Teilmatrix der analytischen Jacobi-Matrix

% %VERSIONINFO%

function Ja_rot = %FN%(q, link_index, ...
  pkin)
