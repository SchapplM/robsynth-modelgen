% Calculate joint inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_MR%
% %INPUT_IF%
% 
% Output:
% Mq [%NQJ%x%NQJ%]
%   inertia matrix

% %VERSIONINFO%

function Mq = %FN%(qJ, ...
  pkin, m, mrSges, Ifges)
