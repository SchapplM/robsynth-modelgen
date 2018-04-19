% Geometrischen Jacobi-Matrix für Segment Nr. %LIJAC% (0=Basis) von
% %RN%
% Use Code from Maple symbolic Code Generation
%
% geometrische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
%
% Input:
% %INPUT_QJ%
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% %INPUT_PKIN%
%
% Output:
% Jg [3x%NQJ%]
%   Geometrischen Jacobi-Matrix

% %VERSIONINFO%

function Jg = %FN%(qJ, r_i_i_C, ...
  pkin)


Ja_transl = %RN%_jacobia_transl_%LIJAC%_floatb_twist_sym_varpar(qJ, r_i_i_C, ...
  pkin);
Jg_rot = %RN%_jacobig_rot_%LIJAC%_floatb_twist_sym_varpar(qJ, ...
  pkin);

Jg = [Ja_transl; Jg_rot];
