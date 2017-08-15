% Calculate time derivative of joint inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_PHIB%
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_MR%
% %INPUT_IF%
% 
% Output:
% MqD [%NQJ%x%NQJ%]
%   time derivative of inertia matrix

% %VERSIONINFO%

function Mq = %FN%(q, qD, ...
  pkin, m_num, mrSges_num_mdh, Ifges_num_mdh)
