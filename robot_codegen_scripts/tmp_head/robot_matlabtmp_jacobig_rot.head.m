% Rotatorische Teilmatrix der geometrischen Jacobi-Matrix für Segment Nr. %LIJAC% von 
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% geometrische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
% 
% 
% Input:
% q [%NQJ%x1]
%   Generalized coordinates (joint angles) (generalized coordinates) [rad]
% a_mdh, d_mdh, q_offset_mdh [%NJ%x1]
%   kinematic parameters
% 
% Output:
% Jg_rot [3x%NQJ%]
%   Rotatorische Teilmatrix der geometrischen Jacobi-Matrix

% %VERSIONINFO%

function Jg_rot = %FN%(q, ...
  alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh%KCPARG%)
