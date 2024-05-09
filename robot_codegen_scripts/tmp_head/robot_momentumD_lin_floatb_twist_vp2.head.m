% Calculate linear momentum time derivative for 
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_QJDD%
% V_base [6x1]
%   Base Velocity (twist: stacked translational and angular velocity) in base frame
% A_base [6x1]
%   Base Acceleration (twist: stacked translational and angular acceleration) in base frame
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_R%
% 
% Output:
% PD [3x1]
%   Time derivative of linear momentum

% %VERSIONINFO%

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
% (c) Institut für Regelungstechnik, Universität Hannover
 
function PD = %FN%(qJ, qJD, qJDD, V_base, A_base, pkin, m, mrSges)

