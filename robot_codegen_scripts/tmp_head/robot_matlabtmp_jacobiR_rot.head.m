% Rotatorische Teilmatrix der Rotationsmatrix-Jacobi-Matrix für Segment Nr. %LIJAC% (0=Basis) von
% %RN%
% Use Code from Maple symbolic Code Generation
%
% Rotationsmatrix-Jacobi-Matrix: Differentieller Zusammenhang zwischen
% gestapelter Endeffektor-Rotationsmatrix und verallgemeinerten Koordinaten.
%
%
% Input:
% %INPUT_QJ%
% %INPUT_PKIN%
%
% Output:
% JR_rot [9x%NQJ%]
%   Jacobi-Matrix der Endeffektor-Rotationsmatrix

% %VERSIONINFO%

function JR_rot = %FN%(qJ, ...
  pkin)
