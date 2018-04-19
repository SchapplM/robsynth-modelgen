% Zeitableitung der translatorischen Teilmatrix der analytischen Jacobi-Matrix für Segment Nr. %LIJAC% (0=Basis) von
% %RN%
% Use Code from Maple symbolic Code Generation
%
% analytische Jacobi-Matrix: Differentieller Zusammenhang zwischen
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
% JaD_transl [3x%NQJ%]
%   Zeitableitung der translatorischen Teilmatrix der analytischen Jacobi-Matrix

% %VERSIONINFO%

function JaD_transl = %FN%(qJ, qJD, r_i_i_C, ...
  pkin)
