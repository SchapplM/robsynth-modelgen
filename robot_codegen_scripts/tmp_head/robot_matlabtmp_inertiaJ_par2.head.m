% Calculate joint inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_Q%
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_MR%
% %INPUT_IF%
% 
% Output:
% Mq [%NQJ%x%NQJ%]
%   inertia matrix

% %VERSIONINFO%

function Mq = %FN%(q, ...
  pkin, m_num, mrSges_num_mdh, Ifges_num_mdh)
