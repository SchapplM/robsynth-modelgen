% Calculate time derivative of joint inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_PHIB%
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_R%
% %INPUT_IC%
% 
% Output:
% MqD [%NQJ%x%NQJ%]
%   time derivative of inertia matrix

% %VERSIONINFO%

function Mq = %FN%1(q, qD, ...
  pkin, m_num, rSges_num_mdh, Icges_num_mdh)
