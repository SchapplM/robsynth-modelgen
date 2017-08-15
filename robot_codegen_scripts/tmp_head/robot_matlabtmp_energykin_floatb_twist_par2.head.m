% Calculate kinetic energy for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_QD%
% V_base [6x1]
%   Base Velocity (twist: stacked translational and angular velocity) in base frame
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_MR%
% %INPUT_IF%
% 
% Output:
% T [1x1]
%   kinetic energy

% %VERSIONINFO%

function T = %FN%(qJ, qJD, V_base, ...
  pkin, m_num, mrSges_num_mdh, Ifges_num_mdh)
