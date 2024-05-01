% Calculate centroidal momentum matrix for 
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_RB%
% phi_base [3x1]
%   Base orientation in world frame. Expressed with RPY Euler angles (xyz)
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_R%
% %INPUT_IC%
% 
% Output:
% K [6x(6+%NQJ%)]
%   centroidal momentum matrix (derivative of momentum w.r.t. generalized velocities)
%   Rows: linear momentum, angular momentum
%   Columns: Base velocity (linear, euler angles), joint velocities
% 
% Sources:
% [OrinGos2008] Orin, D.E. and Goswami, A.: Centroidal Momentum Matrix of a
% humanoid robot: Structure and properties (2008) 

% %VERSIONINFO%

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
% (c) Institut für Regelungstechnik, Universität Hannover
 
function K = %FN%(qJ, r_base, pkin, m, rSges, Icges)

