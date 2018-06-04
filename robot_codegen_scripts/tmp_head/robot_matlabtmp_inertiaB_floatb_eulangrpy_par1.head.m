% Calculate floating base base inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_PHIB%
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_R%
% %INPUT_IC%
% 
% Output:
% Mb [6x6]
%   base inertia matrix (gives inertial forces on the base from base acceleration)

% %VERSIONINFO%

function Mb = %FN%(qJ, phi_base, ...
  pkin, m, rSges, Icges)
