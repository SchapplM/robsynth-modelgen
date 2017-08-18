% Calculate joint-base inertia matrix for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_PHIB%
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_R%
% %INPUT_IC%
% 
% Output:
% Mqb [%NQJ%x6]
%   joint-base inertia matrix (gives inertial forces on the joints from base acceleration)

% %VERSIONINFO%

function Mqb = %FN%(qJ, phi_base, ...
  pkin, m_num, rSges_num_mdh, Icges_num_mdh)
