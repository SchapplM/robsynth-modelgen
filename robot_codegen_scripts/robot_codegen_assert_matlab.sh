#!/bin/bash 
# Erstelle temporäre Variablen mit Code-Schnipseln, die für die Erzeugung von Matlab-Code benötigt wird.
# Code-Schnipsel enthalten assert-Befehle
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/../
tmp_pfad=$repo_pfad/robot_codegen_scripts/tmp/
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Schnipsel für Matlab-varpar-Dateien vorbereiten
# Gelenkwinkel
echo "assert(isa(q,'double') && isreal(q) && all(size(q) == [$robot_NJ 1]), ...
  'q has to be [${robot_NJ}x1] double');)" > $tmp_pfad/robot_matlabtmp_assert_q.m
echo "assert(isa(qD,'double') && isreal(qD) && all(size(qD) == [$robot_NJ 1]), ...
  'qD has to be [${robot_NJ}x1] double');)" > $tmp_pfad/robot_matlabtmp_assert_qD.m
echo "assert(isa(qDD,'double') && isreal(qDD) && all(size(qDD) == [$robot_NJ 1]), ...
  'qDD has to be [${robot_NJ}x1] double');)" > $tmp_pfad/robot_matlabtmp_assert_qDD.m

# Basis
echo "assert(isa(g_base,'double') && isreal(g_base) && all(size(g_base) == [3 1]) ...
  'g_base has to be [3x1] double')" > $tmp_pfad/robot_matlabtmp_assert_g.m
echo "assert(isa(r_base,'double') && isreal(r_base) && all(size(r_base) == [3 1]) ...
  'r_base has to be [3x1] double')" > $tmp_pfad/robot_matlabtmp_assert_rB.m
echo "assert(isa(V_base,'double') && isreal(V_base) && all(size(V_base) == [6 1]) ...
  'V_base has to be [6x1] double')" > $tmp_pfad/robot_matlabtmp_assert_vB.m
echo "assert(isa(VD_base,'double') && isreal(VD_base) && all(size(VD_base) == [6 1]) ...
  'VD_base has to be [6x1] double')" > $tmp_pfad/robot_matlabtmp_assert_vBD.m

# Kinematikparameter
echo "assert(isa(a_mdh,'double') && isreal(a_mdh) && all(size(a_mdh) == [$robot_NJ 1]), ...
  'a_mdh has to be [${robot_NJ}x1] double');" > $tmp_pfad/robot_matlabtmp_assert_mdh.m
echo "assert(isa(a_mdh,'double') && isreal(d_mdh) && all(size(d_mdh) == [$robot_NJ 1]), ...
  'd_mdh has to be [${robot_NJ}x1] double');" >> $tmp_pfad/robot_matlabtmp_assert_mdh.m

# Dynamikparameter
echo "assert(isa(rSges_num_mdh,'double') && isreal(rSges_num_mdh) && all(size(rSges_num_mdh) == [$robot_NL,3]), ...
  'rSges_num_mdh has to be [${robot_NL}x3] double');" > $tmp_pfad/robot_matlabtmp_assert_rcom.m
echo "assert(isa(mrSges_num_mdh,'double') && isreal(mrSges_num_mdh) && all(size(mrSges_num_mdh) == [$robot_NL,3]), ...
  'mrSges_num_mdh has to be [${robot_NL}x3] double');" > $tmp_pfad/robot_matlabtmp_assert_mrcom.m
echo "assert(isa(m_num_mdh,'double') && isreal(m_num_mdh) && all(size(m_num_mdh) == [$robot_NL 1]), ...
  'm_num_mdh has to be [${robot_NL}x1] double'); " > $tmp_pfad/robot_matlabtmp_assert_m.m
echo "assert(isa(Icges_num_mdh,'double') && isreal(Icges_num_mdh) && all(size(Icges_num_mdh) == [$robot_NL 6]), ...
  'Icges_num_mdh has to be [${robot_NL}x6] double'); " > $tmp_pfad/robot_matlabtmp_assert_Ic.m
echo "assert(isa(Ifges_num_mdh,'double') && isreal(Ifges_num_mdh) && all(size(Ifges_num_mdh) == [$robot_NL 6]), ...
  'Ifges_num_mdh has to be [${robot_NL}x6] double'); " > $tmp_pfad/robot_matlabtmp_assert_If.m
