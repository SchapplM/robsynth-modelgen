% Jacobian of explicit kinematic constraints of
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_PKIN%
% 
% Output:
% W [(%NJ%-%NQJ%)x%NQJ%] 
%
% Sources:
% [NakamuraGho1989] Nakamura, Yoshihiko and Ghodoussi, Modjtaba: Dynamics computation of closed-link robot mechanisms with nonredundant and redundant actuators (1989)

% %VERSIONINFO%

function W = %FN%(qJ, pkin)
