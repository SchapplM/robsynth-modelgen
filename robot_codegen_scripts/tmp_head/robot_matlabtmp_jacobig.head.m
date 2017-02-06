% Geometrischen Jacobi-Matrix für Segment Nr. %LIJAC% von
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% geometrische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) (generalized coordinates) [rad]
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% a_mdh, d_mdh, q_offset_mdh, ... [%NJ%x1]
%   kinematic parameters
% 
% Output:
% Jg [3x%NQJ%]
%   Geometrischen Jacobi-Matrix

% %VERSIONINFO%

function Jg = %FN%(q, r_i_i_C, ...
  pkin)


Ja_transl = %RN%_jacobia_transl_%LIJAC%_floatb_twist_sym_varpar(q, r_i_i_C, ...
  pkin);
Jg_rot = %RN%_jacobig_rot_%LIJAC%_floatb_twist_sym_varpar(q, ...
  pkin);

Jg = [Ja_transl; Jg_rot];
