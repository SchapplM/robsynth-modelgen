% Zeitableitung der analytischen Jacobi-Matrix für Segment Nr. %LIJAC% von
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
% qD [%NQJ%x1]
%   Generalized velocities (joint velocities) [rad/s]
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% a_mdh, d_mdh, q_offset_mdh, ... [%NJ%x1]
%   kinematic parameters
% 
% Output:
% JaD [6x%NQJ%]
%   Zeitableitung der analytischen Jacobi-Matrix

% %VERSIONINFO%

function JaD = %FN%(q, qD, r_i_i_C, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh%KCPARG%)

JaD_transl = %RN%_jacobiaD_transl_%LIJAC%_floatb_twist_sym_varpar(q, qD, r_i_i_C, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh%KCPARG%);
JaD_rot = %RN%_jacobiaD_rot_%LIJAC%_floatb_twist_sym_varpar(q, qD, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh%KCPARG%);

JaD = [JaD_transl; JaD_rot];
