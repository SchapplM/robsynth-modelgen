#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Kinematik IC"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
# Initialisiere Variablen
source robot_codegen_tmpvar_bash_ic.sh
source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh

# Funktionen für implizite kinematische Zwangsbedingungen
quelldat=$repo_pfad/codeexport/${robot_name}/tmp/kinconstr_impl_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_kinconstr_impl_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_kinconstr_impl.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess_ic.sh $zieldat 1 1 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

quelldat=$repo_pfad/codeexport/${robot_name}/tmp/kinconstr_impl_active_jacobian_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_kinconstr_impl_act_jacobian_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_kinconstr_impl_active_jacobian.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess_ic.sh $zieldat 1 0 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

quelldat=$repo_pfad/codeexport/${robot_name}/tmp/kinconstr_impl_active_jacobianD_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_kinconstr_impl_act_jacobianD_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_kinconstr_impl_active_jacobianD.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess_ic.sh $zieldat 1 0 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

quelldat=$repo_pfad/codeexport/${robot_name}/tmp/kinconstr_impl_passive_jacobian_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_kinconstr_impl_pas_jacobian_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_kinconstr_impl_passive_jacobian.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess_ic.sh $zieldat 1 0 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

quelldat=$repo_pfad/codeexport/${robot_name}/tmp/kinconstr_impl_passive_jacobianD_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_kinconstr_impl_pas_jacobianD_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_kinconstr_impl_passive_jacobianD.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess_ic.sh $zieldat 1 0 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

quelldat=$repo_pfad/codeexport/${robot_name}/tmp/positionVector_NQJ_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_positionVector_NQJ.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_position_vector.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  sed -i "s/%RN%/$robot_name/g" $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 1 ${quelldat}.subsvar
  sed -i "s/%NAJ%/$robot_NAJ/g" $zieldat
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

quelldat=$repo_pfad/codeexport/${robot_name}/tmp/kinconstr_impl_projection_jacobian_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_kinconstr_impl_projection_jacobian_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_kinconstr_impl_jacobian.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess_ic.sh $zieldat 1 0 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi
