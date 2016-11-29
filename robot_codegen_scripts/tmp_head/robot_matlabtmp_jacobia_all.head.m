% Analytische Jacobi-Matrix für beliebiges Segment von
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
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% a_mdh, d_mdh, q_offset_mdh, ... [%NJ%x1]
%   kinematic parameters
% 
% Output:
% Ja [6x%NQJ%]
%   Analytische Jacobi-Matrix

% %VERSIONINFO%

function Ja = %FN%(q, link_index, r_i_i_C, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh%KCPARG%)


