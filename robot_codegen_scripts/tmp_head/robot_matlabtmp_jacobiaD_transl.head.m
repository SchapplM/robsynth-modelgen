% Zeitableitung der translatorischen Teilmatrix der analytischen Jacobi-Matrix für Segment Nr. %LIJAC% (1=Basis) von 
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% analytische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
% 
% Input:
% %INPUT_Q%
% %INPUT_QD%
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
