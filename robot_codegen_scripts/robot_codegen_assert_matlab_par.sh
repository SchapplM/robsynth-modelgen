#!/bin/bash -e
# Erstelle temporäre Variablen mit Code-Schnipseln, die für die Erzeugung von Matlab-Code benötigt wird.
# Code-Schnipsel enthalten assert-Befehle
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/../
tmp_pfad=$repo_pfad/workdir/tmp
source $repo_pfad/robot_codegen_definitions/robot_env_par.sh
source $repo_pfad/codeexport/${robot_leg_name}/tmp/robot_env.sh

# Schnipsel für Matlab-varpar-Dateien vorbereiten
# Parallel Roboter
echo "assert(isreal(xP) && all(size(xP) == [$parallel_NX 1]), ...
  '%FN%: xP has to be [${parallel_NX}x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_xP.m
echo "assert(isreal(xDP) && all(size(xDP) == [$parallel_NX 1]), ...
  '%FN%: xDP has to be [${parallel_NX}x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_xDP.m
echo "assert(isreal(xDDP) && all(size(xDDP) == [$parallel_NX 1]), ...
  '%FN%: xDDP has to be [${parallel_NX}x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_xDDP.m
  
# Gelenkwinkel
echo "assert(isreal(qJ) && all(size(qJ) == [$parallel_NQJ_leg $parallel_NLEGS]), ...
  '%FN%: qJ has to be [${parallel_NQJ_leg}x${parallel_NLEGS}] (double)');" > $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m

# Kinematikparameter
echo "assert(isreal(pkin) && all(size(pkin) == [$robot_NKP 1]), ...
  '%FN%: pkin has to be [${robot_NKP}x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_KP.m
  
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
  
