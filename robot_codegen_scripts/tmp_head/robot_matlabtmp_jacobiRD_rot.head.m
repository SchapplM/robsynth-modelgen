% Zeitableitung der Rotationsmatrix-Jacobi-Matrix für Segment Nr. %LIJAC% (0=Basis) von
% %RN%
% Use Code from Maple symbolic Code Generation
%
% Rotationsmatrix-Jacobi-Matrix: Differentieller Zusammenhang zwischen
% gestapelter Endeffektor-Rotationsmatrix und verallgemeinerten Koordinaten.
% Zeitableitung: Die Gradientenmatrix wird nochmal nach der Zeit abgeleitet.
%
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_PKIN%
%
% Output:
% JRD_rot [9x%NQJ%]
%   Zeitableitung der Jacobi-Matrix der Endeffektor-Rotationsmatrix

% %VERSIONINFO%

function JRD_rot = %FN%(qJ, qJD, pkin)
