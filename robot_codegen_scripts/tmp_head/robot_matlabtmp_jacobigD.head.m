% Zeitableitung der geometrischen Jacobi-Matrix für Segment Nr. %LIJAC% (0=Basis) von
% %RN%
% Use Code from Maple symbolic Code Generation
%
% geometrische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
%
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% %INPUT_PKIN%
%
% Output:
% JgD [6x%NQJ%]
%   Zeitableitung der geometrischen Jacobi-Matrix

% %VERSIONINFO%

function JgD = %FN%(qJ, qJD, r_i_i_C, ...
  pkin)


JaD_transl = %RN%_jacobiaD_transl_%LIJAC%_sym_varpar(qJ, qJD, r_i_i_C, ...
  pkin);
JgD_rot = %RN%_jacobigD_rot_%LIJAC%_sym_varpar(qJ, qJD, ...
  pkin);

JgD = [JaD_transl; JgD_rot];
