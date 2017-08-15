% Calculate full inertia matrix (base and joints) for
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
% M [(6+%NQJ%)x(6+%NQJ%)]
%   full inertia matrix (for base and joint dynamics)

% %VERSIONINFO%

function M = %FN%(qJ, phi_base, ...
  pkin, m_num, rSges_num_mdh, Icges_num_mdh)
