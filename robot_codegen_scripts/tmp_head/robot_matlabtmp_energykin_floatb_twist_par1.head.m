% Calculate kinetic energy for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% V_base [6x1]
%   Base Velocity (twist: stacked translational and angular velocity) in base frame
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_R%
% %INPUT_IC%
% 
% Output:
% T [1x1]
%   kinetic energy

% %VERSIONINFO%

function T = %FN%(qJ, qJD, V_base, ...
  pkin, m, rSges, Icges)
