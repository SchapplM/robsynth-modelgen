% Calculate centroidal momentum matrix time derivative for 
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% %INPUT_QJ%
% %INPUT_QJD%
% V_base [6x1]
%   Base Velocity (twist: stacked translational and angular velocity) in base frame
% %INPUT_PKIN%
% %INPUT_M%
% %INPUT_MR%
% %INPUT_IF%
% 
% Output:
% KD [6x(6+%NQJ%)]
%   time derivative of centroidal momentum matrix (derivative of momentum w.r.t. velocities) time derivative 
%   Rows: linear momentum, angular momentum
%   Columns: Base velocity (linear, Euler angles), joint velocities
% 
% Sources:
% [OrinGos2008] Orin, D.E. and Goswami, A.: Centroidal Momentum Matrix of a
% humanoid robot: Structure and properties (2008) 
% [LeeGos2011] Lee, S.-H. & Goswami, A.Fall on Backpack: Damage Minimizing
% Humanoid Fall on Targeted Body Segment Using Momentum Control (2011)

% %VERSIONINFO%

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
% (c) Institut für Regelungstechnik, Universität Hannover
 
function KD = %FN%(qJ, qJD, r_base, V_base, pkin, m, mrSges, Ifges)

