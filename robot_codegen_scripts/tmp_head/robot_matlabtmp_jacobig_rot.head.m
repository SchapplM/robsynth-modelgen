% Rotatorische Teilmatrix der geometrischen Jacobi-Matrix für Segment Nr. %LIJAC% (0=Basis) von
% %RN%
% Use Code from Maple symbolic Code Generation
%
% geometrische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
%
%
% Input:
% %INPUT_QJ%
% %INPUT_PKIN%
%
% Output:
% Jg_rot [3x%NQJ%]
%   Rotatorische Teilmatrix der geometrischen Jacobi-Matrix

% %VERSIONINFO%

function Jg_rot = %FN%(qJ, ...
  pkin)
