% Jacobian time derivative of explicit kinematic constraints of
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% %INPUT_PKIN%
% 
% Output:
% WD [(%NJ%-%NQJ%)x%NQJ%] 
%
% Sources:
% [NakamuraGho1989] Nakamura, Yoshihiko and Ghodoussi, Modjtaba: Dynamics computation of closed-link robot mechanisms with nonredundant and redundant actuators (1989)
% [ParkChoPlo1999] Park, FC and Choi, Jihyeon and Ploen, SR: Symbolic formulation of closed chain dynamics in independent coordinates

% %VERSIONINFO%

function WD = %FN%(qJ, qJD, pkin)
