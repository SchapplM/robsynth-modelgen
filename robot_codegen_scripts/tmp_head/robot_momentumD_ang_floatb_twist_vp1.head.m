% Calculate angular momentum time derivative for 
% %RN%
% Use matlab function from symbolic calculations
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_QJDD%
% %INPUT_RB%
% V_base [6x1]
%   Base Velocity (twist: stacked translational and angular velocity) in base frame
% A_base [6x1]
%   Base Acceleration (twist: stacked translational and angular acceleration) in base frame
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_R%
% %INPUT_IC%
% 
% Output:
% LD [3x1]
%   Time derivative of angular momentum

% %VERSIONINFO%

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
% (c) Institut für Regelungstechnik, Universität Hannover
 
function LD = %FN%(qJ, qJD, qJDD, r_base, V_base, A_base, pkin, m, rSges, Icges)

