#!/bin/bash
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Dynamik-Regressor"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/robot_codegen_scripts/tmp/
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Erstelle Matlab-Funktionen der Regressorform

# Minimalparametervektor
quelldat=$repo_pfad/codeexport/${robot_name}_minimal_parameter_vector_matlab.m
zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_convert_par2_MPV.m
if [ -f $quelldat ]; then
  cat $tmp_pfad/robot_matlabtmp_convert_par2_MPV.head.m > $zieldat
  printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
  # echo "%% Conversion Parameterset 1 -> Parameterset 2" >> $zieldat
  # echo "[mrcges, Ifges] = inertial_parameters_convert_par1_par2(rSges, Icges, m);" >> $zieldat

  echo "%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
  
  printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Coriolis-Vektor
quelldat=$repo_pfad/codeexport/${robot_name}_coriolisvec_joint_fixb_regressor_minpar_matlab.m
zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_coriolisvec_joint_fixb_regmin_sym_lag_varpar.m
if [ -f $quelldat ]; then
  cat $tmp_pfad/robot_matlabtmp_coriolisvec_joint_fixb_regmin.head.m > $zieldat
  printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
  printf "\n%%%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
  printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Coriolis-Matrix
quelldat=$repo_pfad/codeexport/${robot_name}_coriolismat_joint_fixb_regressor_minpar_matlab.m
zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_coriolismat_joint_fixb_regmin_sym_lag_varpar.m
if [ -f $quelldat ]; then
  cat $tmp_pfad/robot_matlabtmp_coriolismat_joint_fixb_regmin.head.m > $zieldat
  printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
  printf "\n%%%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
  printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Massenmatrix
quelldat=$repo_pfad/codeexport/${robot_name}_inertia_joint_joint_regressor_minpar_matlab.m
zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_inertia_joint_regmin_sym_lag_varpar.m
if [ -f $quelldat ]; then
  cat $tmp_pfad/robot_matlabtmp_inertia_joint_regmin.head.m > $zieldat
  printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
  printf "\n%%%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
  printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Massenmatrix-Zeitableitung
quelldat=$repo_pfad/codeexport/${robot_name}_inertiaD_joint_joint_regressor_minpar_matlab.m
zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_inertiaD_joint_regmin_sym_lag_varpar.m
if [ -f $quelldat ]; then
  cat $tmp_pfad/robot_matlabtmp_inertiaD_joint_regmin.head.m > $zieldat
  printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
  printf "\n%%%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
  printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Gravitationsmoment
quelldat=$repo_pfad/codeexport/${robot_name}_gravload_joint_regressor_minpar_matlab.m
zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_gravload_joint_regmin_sym_lag_varpar.m
if [ -f $quelldat ]; then
  cat $tmp_pfad/robot_matlabtmp_gravload_joint_regmin.head.m > $zieldat
  printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
  printf "\n%%%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_g.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
  printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Kinetische Energie (Fixed Base)
quelldat=$repo_pfad/codeexport/${robot_name}_energy_kinetic_fixb_regressor_minpar_matlab.m
zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_energykin_fixb_regmin_sym_lag_varpar.m
if [ -f $quelldat ]; then
  cat $tmp_pfad/robot_matlabtmp_energykin_fixb_regmin.head.m > $zieldat
  printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
  printf "\n%%%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
  printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi


# Potentielle Energie (Fixed base)
quelldat=$repo_pfad/codeexport/${robot_name}_energy_potential_fixb_regressor_minpar_matlab.m
zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_energypot_fixb_regmin_sym_lag_varpar.m
if [ -f $quelldat ]; then
  cat $tmp_pfad/robot_matlabtmp_energypot_fixb_regmin.head.m > $zieldat
  printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
  printf "\n%%%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_g.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
  printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi
