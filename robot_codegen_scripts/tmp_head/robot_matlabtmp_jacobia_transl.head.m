% Translatorische Teilmatrix der analytischen Jacobi-Matrix für Segment Nr. %LIJAC% (0=Basis) von
% %RN%
% Use Code from Maple symbolic Code Generation
%
% analytische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
%
% Input:
% %INPUT_QJ%
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% %INPUT_PKIN%
%
% Output:
% Ja_transl [3x%NQJ%]
%   Translatorische Teilmatrix der analytischen Jacobi-Matrix

% %VERSIONINFO%

function Ja_transl = %FN%(qJ, r_i_i_C, ...
  pkin)
