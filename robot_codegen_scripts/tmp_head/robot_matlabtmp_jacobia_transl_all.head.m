% Analytische Jacobi-Matrix (Translatorisch) für beliebiges Segment von
% %RN%
% 
% analytische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
% (Ist für translatorischen Teil egal, kennzeichnet nur den Rechenweg der Herleitung)
% 
% Input:
% %INPUT_QJ%
% link_index [1x1 uint8]
%   Index des Segmentes, auf dem der Punkt C liegt (0=Basis).
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% %INPUT_PKIN%
% 
% Output:
% Ja_transl [3x%NQJ%]
%   Translatorischer Teil der analytischen Jacobi-Matrix

% %VERSIONINFO%

function Ja_transl = %FN%(qJ, link_index, r_i_i_C, ...
  pkin)


