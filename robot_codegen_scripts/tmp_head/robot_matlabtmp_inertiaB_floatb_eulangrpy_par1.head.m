% Calculate floating base base inertia matrix for
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
% Mb [6x6]
%   base inertia matrix (gives inertial forces on the base from base acceleration)

% %VERSIONINFO%

function Mb = %FN%(qJ, phi_base, ...
  pkin, m_num, rSges_num_mdh, Icges_num_mdh)
