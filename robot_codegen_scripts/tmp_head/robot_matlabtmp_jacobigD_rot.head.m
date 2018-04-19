% Zeitableitung der rotatorischen Teilmatrix der geometrischen Jacobi-Matrix für Segment Nr. %LIJAC% (0=Basis) von
% %RN%
% Use Code from Maple symbolic Code Generation
%
% geometrische Jacobi-Matrix: Differentieller Zusammenhang zwischen
% Endeffektorposition und verallgemeinerten Koordinaten.
%
%
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_PKIN%
%
% Output:
% JgD_rot [3x%NQJ%]
%   Zeitableitung der rotatorischen Teilmatrix der geometrischen Jacobi-Matrix

% %VERSIONINFO%

function JgD_rot = %FN%(qJ, qJD, ...
  pkin)
