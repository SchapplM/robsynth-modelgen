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
% Mq [%NAJ%x%NAJ%]
%   inertia matrix

% %VERSIONINFO%

function Mq = %FN%(qJ, ...
  pkin, m, rSges, Icges)
