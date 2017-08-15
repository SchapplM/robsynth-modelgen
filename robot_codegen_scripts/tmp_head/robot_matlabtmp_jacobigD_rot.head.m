% Zeitableitung der rotatorischen Teilmatrix der geometrischen Jacobi-Matrix für Segment Nr. %LIJAC% (1=Basis) von 
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% geometrische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
% 
% 
% Input:
% %INPUT_Q%
% %INPUT_QD%
% %INPUT_PKIN%
% 
% Output:
% JgD_rot [3x%NQJ%]
%   Zeitableitung der rotatorischen Teilmatrix der geometrischen Jacobi-Matrix

% %VERSIONINFO%

function JgD_rot = %FN%(q, qD, ...
  pkin)
