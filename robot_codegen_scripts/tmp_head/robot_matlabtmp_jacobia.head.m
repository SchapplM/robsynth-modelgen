% Analytische Jacobi-Matrix für Segment Nr. %LIJAC% (0=Basis) von
% %RN%
% Use Code from Maple symbolic Code Generation
%
% analytische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
% Zeitableitung der Winkeldarstellung des Endeffektors in Basis-Koordinaten
%
% Winkeldarstellung: Euler-XYZ-Winkel, rotx(alpha)*roty(beta)*rotz(gamma)
%
% Input:
% %INPUT_QJ%
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% %INPUT_PKIN%
%
% Output:
% Ja [6x%NQJ%]
%   Analytischen Jacobi-Matrix

% %VERSIONINFO%

function Ja = %FN%(qJ, r_i_i_C, ...
  pkin)

Ja_transl = %RN%_jacobia_transl_%LIJAC%_sym_varpar(qJ, r_i_i_C, ...
  pkin);
Ja_rot = %RN%_jacobia_rot_%LIJAC%_sym_varpar(qJ, ...
  pkin);

Ja = [Ja_transl; Ja_rot];
