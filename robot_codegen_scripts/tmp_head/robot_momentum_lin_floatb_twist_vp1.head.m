% Calculate linear momentum for 
% %RN%
% Use matlab function from symbolic calculations
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% V_base [6x1]
%   Base Velocity (twist: stacked translational and angular velocity) in base frame
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_R%
% 
% Output:
% P [3x1]
%   Linear Momentum

% %VERSIONINFO%

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
% (c) Institut für Regelungstechnik, Universität Hannover
 
function P = %FN%(qJ, qJD, V_base, pkin, m, rSges)

