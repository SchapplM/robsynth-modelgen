% Zeitableitung der analytischen Jacobi-Matrix für Segment Nr. %LIJAC% (1=Basis) von
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
% %INPUT_QJ%
% %INPUT_QJD%
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% %INPUT_PKIN%
% 
% Output:
% JaD [6x%NQJ%]
%   Zeitableitung der analytischen Jacobi-Matrix

% %VERSIONINFO%

function JaD = %FN%(qJ, qJD, r_i_i_C, ...
  pkin)

JaD_transl = %RN%_jacobiaD_transl_%LIJAC%_floatb_twist_sym_varpar(qJ, qJD, r_i_i_C, ...
  pkin);
JaD_rot = %RN%_jacobiaD_rot_%LIJAC%_floatb_twist_sym_varpar(qJ, qJD, ...
  pkin);

JaD = [JaD_transl; JaD_rot];
