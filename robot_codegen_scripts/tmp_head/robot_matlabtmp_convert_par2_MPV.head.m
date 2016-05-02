% Return the minimum parameter vector for
% %RN%
% Use Code from Maple symbolic Code Generation
% 
% Input:
% a_mdh, d_mdh [%NJ%x1]
%   kinematic parameters
% m_num_mdh, mrSges_num_mdh, Ifges_num_mdh [%NL%x1]
%   dynamic parameters (parameter set 2: first moment and inertia about link frame origin)
% 
% Output:
% MPV [%NMPV%x1]
%   base parameter vector (minimal parameter vector)

function MPV = %RN%_convert_par2_MPV(alpha_mdh, a_mdh, d_mdh, q_offset_mdh, b_mdh, beta_mdh, m_num, mrSges_num_mdh, Ifges_num_mdh)
