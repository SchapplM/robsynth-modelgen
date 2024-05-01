% Calculate angular momentum for 
% %RN%
% Use matlab function from symbolic calculations
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_RB%
% V_base [6x1]
%   Base Velocity (twist: stacked translational and angular velocity) in base frame
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_MR%
% %INPUT_IF%
% 
% Output:
% L [3x1]
%   Angular Momentum

% %VERSIONINFO%

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
% (c) Institut für Regelungstechnik, Universität Hannover
 
function P = %FN%(qJ, qJD, r_base, V_base, pkin, m, mrSges, Ifges)

