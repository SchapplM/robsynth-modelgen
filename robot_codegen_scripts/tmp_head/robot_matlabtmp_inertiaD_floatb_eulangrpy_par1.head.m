% Calculate inertia matrix time derivative for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_RB%
% %INPUT_PHIB%
% %INPUT_XDB%
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_R%
% %INPUT_IC%
% 
% Output:
% MD [(6+%NQJ%)x(6+%NQJ%)]
%   full time derivative of inertia matrix (for base and joint dynamics)

% %VERSIONINFO%

function MD = %FN%(qJ, qJD, phi_base, xD_base, ...
  pkin, m, rSges, Icges)
