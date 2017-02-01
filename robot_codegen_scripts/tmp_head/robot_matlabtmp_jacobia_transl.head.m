% Translatorische Teilmatrix der analytischen Jacobi-Matrix für Segment Nr. %LIJAC% von 
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% analytische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) (generalized coordinates) [rad]
% r_i_i_C [3x1]
%   Ortsvektor vom KörperKS-Ursprung zum gesuchten Punkt
% a_mdh, d_mdh, q_offset_mdh, ... [%NJ%x1]
%   kinematic parameters
% 
% Output:
% Ja_transl [3x%NQJ%]
%   Translatorische Teilmatrix der analytischen Jacobi-Matrix

% %VERSIONINFO%

function Ja_transl = %FN%(q, r_i_i_C, ...
  pkin)
