% Zeitableitung der geometrischen Jacobi-Matrix für Segment Nr. %LIJAC% von
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% geometrische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
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
% JgD [6x%NQJ%]
%   Zeitableitung der geometrischen Jacobi-Matrix

% %VERSIONINFO%

function JgD = %FN%(q, qD, r_i_i_C, ...
  pkin)


JaD_transl = %RN%_jacobiaD_transl_%LIJAC%_floatb_twist_sym_varpar(q, qD, r_i_i_C, ...
  pkin);
JgD_rot = %RN%_jacobigD_rot_%LIJAC%_floatb_twist_sym_varpar(q, qD, ...
  pkin);

JgD = [JaD_transl; JgD_rot];
