#!/bin/bash -e
# Erstelle temporäre Variablen mit Code-Schnipseln, die für die Erzeugung von Matlab-Code benötigt wird.
# Code-Schnipsel enthalten assert-Befehle
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/../
tmp_pfad=$repo_pfad/robot_codegen_scripts/tmp/
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Schnipsel für Matlab-varpar-Dateien vorbereiten
# Gelenkwinkel
echo "assert(isa(q,'double') && isreal(q) && all(size(q) == [$robot_NQJ 1]), ...
  '%FN%: q has to be [${robot_NQJ}x1] double');" > $tmp_pfad/robot_matlabtmp_assert_q.m
echo "assert(isa(qD,'double') && isreal(qD) && all(size(qD) == [$robot_NQJ 1]), ...
  '%FN%: qD has to be [${robot_NQJ}x1] double');" > $tmp_pfad/robot_matlabtmp_assert_qD.m
echo "assert(isa(qDD,'double') && isreal(qDD) && all(size(qDD) == [$robot_NQJ 1]), ...
  '%FN%: qDD has to be [${robot_NQJ}x1] double');" > $tmp_pfad/robot_matlabtmp_assert_qDD.m

# Basis
echo "assert(isa(g,'double') && isreal(g) && all(size(g) == [3 1]), ...
  '%FN%: g has to be [3x1] double');" > $tmp_pfad/robot_matlabtmp_assert_g.m
echo "assert(isa(r_base,'double') && isreal(r_base) && all(size(r_base) == [3 1]), ...
  '%FN%: r_base has to be [3x1] double');" > $tmp_pfad/robot_matlabtmp_assert_rB.m
echo "assert(isa(phi_base,'double') && isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  '%FN%: phi_base has to be [3x1] double');" > $tmp_pfad/robot_matlabtmp_assert_phiB.m
echo "assert(isa(V_base,'double') && isreal(V_base) && all(size(V_base) == [6 1]), ...
  '%FN%: V_base has to be [6x1] double');" > $tmp_pfad/robot_matlabtmp_assert_vB.m
echo "assert(isa(A_base,'double') && isreal(A_base) && all(size(A_base) == [6 1]), ...
  '%FN%: A_base has to be [6x1] double');" > $tmp_pfad/robot_matlabtmp_assert_aB.m
echo "assert(isa(xD_base,'double') && isreal(xD_base) && all(size(xD_base) == [6 1]), ...
  '%FN%: xD_base has to be [6x1] double');" > $tmp_pfad/robot_matlabtmp_assert_xDB.m
echo "assert(isa(xDD_base,'double') && isreal(xDD_base) && all(size(xDD_base) == [6 1]), ...
  '%FN%: xDD_base has to be [6x1] double');" > $tmp_pfad/robot_matlabtmp_assert_xDDB.m

# Kinematikparameter
echo "assert(isa(pkin,'double') && isreal(pkin) && all(size(pkin) == [$robot_NKP 1]), ...
  '%FN%: pkin has to be [${robot_NKP}x1] double');" > $tmp_pfad/robot_matlabtmp_assert_KP.m

# Dynamikparameter
echo "assert(isa(rSges_num_mdh,'double') && isreal(rSges_num_mdh) && all(size(rSges_num_mdh) == [$robot_NL,3]), ...
  '%FN%: rSges_num_mdh has to be [${robot_NL}x3] double');" > $tmp_pfad/robot_matlabtmp_assert_rcom.m
echo "assert(isa(mrSges_num_mdh,'double') && isreal(mrSges_num_mdh) && all(size(mrSges_num_mdh) == [$robot_NL,3]), ...
  '%FN%: mrSges_num_mdh has to be [${robot_NL}x3] double');" > $tmp_pfad/robot_matlabtmp_assert_mrcom.m
echo "assert(isa(m_num,'double') && isreal(m_num) && all(size(m_num) == [$robot_NL 1]), ...
  '%FN%: m_num has to be [${robot_NL}x1] double'); " > $tmp_pfad/robot_matlabtmp_assert_m.m
echo "assert(isa(Icges_num_mdh,'double') && isreal(Icges_num_mdh) && all(size(Icges_num_mdh) == [$robot_NL 6]), ...
  '%FN%: Icges_num_mdh has to be [${robot_NL}x6] double'); " > $tmp_pfad/robot_matlabtmp_assert_Ic.m
echo "assert(isa(Ifges_num_mdh,'double') && isreal(Ifges_num_mdh) && all(size(Ifges_num_mdh) == [$robot_NL 6]), ...
  '%FN%: Ifges_num_mdh has to be [${robot_NL}x6] double'); " > $tmp_pfad/robot_matlabtmp_assert_If.m

# Kinematische Zwangsbedingungen
if [ $robot_kinconstr_exist == 1 ]; then
  echo "assert(isa(kintmp,'double') && isreal(kintmp) && all(size(kintmp) == [$robot_NKCP,1]), ...
    '%FN%: kintmp has to be [${robot_NKCP}x1] double');" > $tmp_pfad/robot_matlabtmp_assert_KCP.m
else
  # Es gibt keine kinematischen Zwangsbedingungen, die Datei bleibt leer
  echo "" > $tmp_pfad/robot_matlabtmp_assert_KCP.m
fi;
