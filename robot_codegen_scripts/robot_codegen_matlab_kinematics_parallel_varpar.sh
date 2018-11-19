#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Kinematik PKM"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
# Initialisiere Variablen
source robot_codegen_tmpvar_bash_par.sh
source $repo_pfad/robot_codegen_definitions/robot_env_par.sh

# Inverse Kinematik
quelldat=$repo_pfad/codeexport/${robot_name}/tmp/Jinv_para_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_Jinv.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_jacobia_parallel.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
  source $repo_pfad/scripts/set_inputdim_line_par.sh $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_xP.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_legFrame_parallel.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m >> $zieldat
  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_qJ_parallel.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_xP.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_legFrame_parallel.m >> ${quelldat}.subsvar
  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 1 0 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi;