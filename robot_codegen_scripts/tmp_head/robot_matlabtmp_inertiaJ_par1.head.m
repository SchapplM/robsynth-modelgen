% Calculate joint inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_R%
% %INPUT_IC%
% 
% Output:
% Mq [%NQJ%x%NQJ%]
%   inertia matrix

% %VERSIONINFO%

function Mq = %FN%(qJ, ...
  pkin, m_num, rSges_num_mdh, Icges_num_mdh)
