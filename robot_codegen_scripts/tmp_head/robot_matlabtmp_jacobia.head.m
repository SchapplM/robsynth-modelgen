% Analytischen Jacobi-Matrix für Segment Nr. %LIJAC% von
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
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% pkin [%NKP%x1]
%   kinematic parameters
% 
% Output:
% Ja [6x%NQJ%]
%   Analytischen Jacobi-Matrix

% %VERSIONINFO%

function Ja = %FN%(q, r_i_i_C, ...
  pkin)

Ja_transl = %RN%_jacobia_transl_%LIJAC%_floatb_twist_sym_varpar(q, r_i_i_C, ...
  pkin);
Ja_rot = %RN%_jacobia_rot_%LIJAC%_floatb_twist_sym_varpar(q, ...
  pkin);

Ja = [Ja_transl; Ja_rot];
