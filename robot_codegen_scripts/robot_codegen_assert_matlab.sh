#!/bin/bash -e
# Erstelle temporäre Variablen mit Code-Schnipseln, die für die Erzeugung von Matlab-Code benötigt wird.
# Code-Schnipsel enthalten assert-Befehle
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/../
tmp_pfad=$repo_pfad/workdir/tmp
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Schnipsel für Matlab-varpar-Dateien vorbereiten
# Gelenkwinkel
echo "assert(isreal(qJ) && all(size(qJ) == [$robot_NQJ 1]), ...
  '%FN%: qJ has to be [${robot_NQJ}x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_qJ.m
echo "assert(isreal(qJD) && all(size(qJD) == [$robot_NQJ 1]), ...
  '%FN%: qJD has to be [${robot_NQJ}x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_qJD.m
echo "assert(isreal(qJDD) && all(size(qJDD) == [$robot_NQJ 1]), ...
  '%FN%: qJDD has to be [${robot_NQJ}x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_qJDD.m

# Parallel Roboter
echo "assert(isreal(xP) && all(size(xP) == [$parallel_NX 1]), ...
  '%FN%: xP has to be [${parallel_NX}x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_xP.m
echo "assert(isreal(xPDD) && all(size(xPDD) == [$parallel_NX 1]), ...
  '%FN%: xPD has to be [${parallel_NX}x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_xPD.m
echo "assert(isreal(xPDD) && all(size(xPDD) == [$parallel_NX 1]), ...
  '%FN%: xPDD has to be [${parallel_NX}x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_xPDD.m
# for (( i=1 ; i<=$parallel_NLEGS ; i++ ))
# do
	# echo "assert(isreal(qJ${i}) && all(size(qJ${i}) == [$parallel_NQJ_leg 1]), ...
	  # '%FN%: qJ${i} has to be [${parallel_NQJ_leg}x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m
# done
echo "assert(isreal(qJ) && all(size(qJ) == [$parallel_NQJ_leg $parallel_NLEGS]), ...
  '%FN%: qJ has to be [${parallel_NQJ_leg}x${parallel_NLEGS}] (double)');" > $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m

  
# Basis
echo "assert(isreal(g) && all(size(g) == [3 1]), ...
  '%FN%: g has to be [3x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_g.m
echo "assert(isreal(r_base) && all(size(r_base) == [3 1]), ...
  '%FN%: r_base has to be [3x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_rB.m
echo "assert(isreal(phi_base) && all(size(phi_base) == [3 1]), ...
  '%FN%: phi_base has to be [3x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_phiB.m
echo "assert(isreal(V_base) && all(size(V_base) == [6 1]), ...
  '%FN%: V_base has to be [6x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_vB.m
echo "assert(isreal(A_base) && all(size(A_base) == [6 1]), ...
  '%FN%: A_base has to be [6x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_aB.m
echo "assert(isreal(xD_base) && all(size(xD_base) == [6 1]), ...
  '%FN%: xD_base has to be [6x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_xDB.m
echo "assert(isreal(xDD_base) && all(size(xDD_base) == [6 1]), ...
  '%FN%: xDD_base has to be [6x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_xDDB.m

# Kinematikparameter
echo "assert(isreal(pkin) && all(size(pkin) == [$robot_NKP 1]), ...
  '%FN%: pkin has to be [${robot_NKP}x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_KP.m

# Dynamikparameter
echo "assert(isreal(rSges) && all(size(rSges) == [$robot_NL,3]), ...
  '%FN%: rSges has to be [${robot_NL}x3] (double)');" > $tmp_pfad/robot_matlabtmp_assert_rcom.m
echo "assert(isreal(mrSges) && all(size(mrSges) == [$robot_NL,3]), ...
  '%FN%: mrSges has to be [${robot_NL}x3] (double)');" > $tmp_pfad/robot_matlabtmp_assert_mrcom.m
echo "assert(isreal(m) && all(size(m) == [$robot_NL 1]), ...
  '%FN%: m has to be [${robot_NL}x1] (double)'); " > $tmp_pfad/robot_matlabtmp_assert_m.m
echo "assert(isreal(Icges) && all(size(Icges) == [$robot_NL 6]), ...
  '%FN%: Icges has to be [${robot_NL}x6] (double)'); " > $tmp_pfad/robot_matlabtmp_assert_Ic.m
echo "assert(isreal(Ifges) && all(size(Ifges) == [$robot_NL 6]), ...
  '%FN%: Ifges has to be [${robot_NL}x6] (double)'); " > $tmp_pfad/robot_matlabtmp_assert_If.m
  
# Dynamikparameter für parallele Roboter
alleKoerper=$((parallel_NQJ_leg + 1))
echo "assert(isreal(rSges) && all(size(rSges) == [$alleKoerper,3]), ...
  '%FN%: rSges has to be [${alleKoerper}x3] (double)');" > $tmp_pfad/robot_matlabtmp_assert_rcom_parallel.m
echo "assert(isreal(mrSges) && all(size(mrSges) == [$alleKoerper,3]), ...
  '%FN%: mrSges has to be [${alleKoerper}x3] (double)');" > $tmp_pfad/robot_matlabtmp_assert_mrcom_parallel.m
echo "assert(isreal(m) && all(size(m) == [$alleKoerper 1]), ...
  '%FN%: m has to be [${alleKoerper}x1] (double)'); " > $tmp_pfad/robot_matlabtmp_assert_m_parallel.m
echo "assert(isreal(Icges) && all(size(Icges) == [$alleKoerper 6]), ...
  '%FN%: Icges has to be [${alleKoerper}x6] (double)'); " > $tmp_pfad/robot_matlabtmp_assert_Ic_parallel.m
echo "assert(isreal(Ifges) && all(size(Ifges) == [$alleKoerper 6]), ...
  '%FN%: Ifges has to be [${alleKoerper}x6] (double)'); " > $tmp_pfad/robot_matlabtmp_assert_If_parallel.m

# Orientierungen der Basis-KOs für paralelle Roboter
echo "assert(isreal(legFrame) && all(size(legFrame) == [$parallel_NLEGS 3]), ...
  '%FN%: legFrame has to be [${parallel_NLEGS}x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_legFrame_parallel.m

# Koppelpunkte Plattform
echo "assert(isreal(koppelP) && all(size(koppelP) == [$parallel_NLEGS 3]), ...
  '%FN%: Koppelpunkt has to be [${parallel_NLEGS}x3] (double)');" > $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m
  
# Schwerpunkt Plattform
echo "assert(isreal(rSP) && all(size(rSP) == [3 1]), ...
  '%FN%: Plattformschwerpunkt has to be [3x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_rSP_parallel.m
  
# Kinematische Zwangsbedingungen
if [ $robot_kinconstr_exist == 1 ]; then
  echo "assert(isreal(kintmp) && all(size(kintmp) == [$robot_NKCP,1]), ...
    '%FN%: kintmp has to be [${robot_NKCP}x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_KCP.m
else
  # Es gibt keine kinematischen Zwangsbedingungen, die Datei bleibt leer
  echo "" > $tmp_pfad/robot_matlabtmp_assert_KCP.m
fi;
