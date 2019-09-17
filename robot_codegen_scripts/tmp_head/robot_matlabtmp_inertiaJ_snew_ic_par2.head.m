% Calculate joint inertia matrix with Newton Euler for
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
% Mq [%NAJ%x%NAJ%]
%   inertia matrix

% %VERSIONINFO%

function Mq = %FN%(qJ, ...
  pkin, m, mrSges, Ifges)
