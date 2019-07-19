#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Dynamik-Regressor (Fixed Base)"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Erstelle Matlab-Funktionen der Regressorform

# Minimalparametervektor (Fixed Base)
quelldat=$repo_pfad/codeexport/${robot_name}/tmp/minimal_parameter_vector_fixb_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_convert_par2_MPV_fixb.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_convert_par2_MPV_fixb.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
  source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
  # echo "%% Conversion Parameterset 1 -> Parameterset 2" >> $zieldat
  # echo "[mrcges, Ifges] = inertial_parameters_convert_par1_par2(rSges, Icges, m);" >> $zieldat

  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_m.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_If.m >> ${quelldat}.subsvar

  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Belegungsmatrix des Minimalparametervektors mit den Inertialparametern (Fixed Base)
quelldat1=$repo_pfad/codeexport/${robot_name}/tmp/PV2_MPV_transformation_linear_fixb_matlab.m
quelldat2=$repo_pfad/codeexport/${robot_name}/tmp/PV2_MPV_transformation_linear_dependant_fixb_matlab.m
quelldat3=$repo_pfad/codeexport/${robot_name}/tmp/PV2_permutation_linear_independant_fixb_matlab.m
quelldat4=$repo_pfad/codeexport/${robot_name}/tmp/PV2_permutation_linear_dependant_fixb_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_PV2_MPV_transformations_fixb.m
if [ -f $quelldat1 ] && [ -f $quelldat2 ] && [ -f $quelldat3 ] && [ -f $quelldat4 ]; then
  cat $head_pfad/robot_matlabtmp_PV2_MPV_transformations_fixb.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
  source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
  echo "%% Symbolic Expressions" >> $zieldat
  printf "%% From ${quelldat1##*/}\n" >> $zieldat
  cat $quelldat1 >> $zieldat
  varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $quelldat1`
  echo "K = $varname_tmp;" >> $zieldat
  printf "%% From ${quelldat2##*/}\n" >> $zieldat
  cat $quelldat2 >> $zieldat
  varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $quelldat2`
  echo "K_d = $varname_tmp;" >> $zieldat
  printf "%% From ${quelldat3##*/}\n" >> $zieldat
  cat $quelldat3 >> $zieldat
  varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $quelldat3`
  echo "P_b = $varname_tmp;" >> $zieldat
  printf "%% From ${quelldat4##*/}\n" >> $zieldat
  cat $quelldat4 >> $zieldat
  varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $quelldat4`
  echo "P_d = $varname_tmp;" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0 0 ${quelldat}.subsvar
else
  echo "Code in ${quelldat1##*/} oder anderer nicht gefunden."
fi

# Umwandlung der Invers-Dynamik-Regressor-Matrix in einen Vektor
quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_joint_fixb_regressor_minpar_occupancy_vector_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdynJ_fixb_regmin2vec.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_invdynJ_fixb_regmin2vec.head.m > $zieldat
  printf "%% From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Invers-Dynamik-Funktion mit Multiplikation von Regressormatrix und Parametervektor
quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_joint_fixb_mdp_mult_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdynJ_fixb_mdp_slag_vr.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_invdynJ_fixb_mdp_vr.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  echo "%\$cgargs {zeros(%NTAUJFIXBREGNN%,1), zeros(%NMPVFIXB%,1)}" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_MDPFIXB.m >> $zieldat
  printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_MDPFIXB.m >> ${quelldat}.subsvar
  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi


# Generiere zwei verschiedene Regressorformen:
# Minimalparameterregressor (rm=1) und Inertialparameterregressor (rm=2)
for (( rm=1; rm<=2; rm++ ))
do
  if [ $rm == 1 ]; then
    maple_string_reg="regressor_minpar"
    matlab_string_reg="regmin"
    maple_string_vec="mdp"
    matlab_string_vec="mdp"
  else
    maple_string_reg="regressor"
    matlab_string_reg="reg2"
    maple_string_vec="dp"
    matlab_string_vec="dp"
  fi

  # Coriolis-Vektor Regressor
  quelldat=$repo_pfad/codeexport/${robot_name}/tmp/coriolisvec_joint_fixb_${maple_string_reg}_matlab.m
  zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_coriolisvecJ_fixb_${matlab_string_reg}_slag_vp.m
  if [ -f $quelldat ]; then
    cat $head_pfad/robot_matlabtmp_coriolisvecJ_fixb_${matlab_string_reg}.head.m > $zieldat
    printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
    source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
    printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
    printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
    sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi

  # Coriolis-Vektor als Funktion der Dynamikparameter
  if [ $rm == 1 ]; then
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/coriolisvec_joint_fixb_${maple_string_vec}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_coriolisvecJ_fixb_${matlab_string_vec}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_coriolisvecJ_fixb_${matlab_string_vec}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_MDPFIXB.m >> $zieldat
      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_MDPFIXB.m >> ${quelldat}.subsvar
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi
  fi
  
  # Coriolis-Matrix-Regressor
  quelldat=$repo_pfad/codeexport/${robot_name}/tmp/coriolismat_joint_fixb_${maple_string_reg}_matlab.m
  zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_coriolismatJ_fixb_${matlab_string_reg}_slag_vp.m
  if [ -f $quelldat ]; then
    cat $head_pfad/robot_matlabtmp_coriolismatJ_fixb_${matlab_string_reg}.head.m > $zieldat
    printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
    source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
    printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
    printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
    sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi

  # Massenmatrix-Regressor
  quelldat=$repo_pfad/codeexport/${robot_name}/tmp/inertia_joint_joint_fixb_${maple_string_reg}_matlab.m
  zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_inertiaJ_${matlab_string_reg}_slag_vp.m
  if [ -f $quelldat ]; then
    cat $head_pfad/robot_matlabtmp_inertiaJ_${matlab_string_reg}.head.m > $zieldat
    printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
    source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
    printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
    printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi
  

  # Massenmatrix als Funktion der Dynamikparameter
  quelldat=$repo_pfad/codeexport/${robot_name}/tmp/inertia_joint_joint_fixb_${maple_string_vec}_matlab.m
  zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_inertiaJ_${matlab_string_vec}_slag_vp.m
  if [ $rm == 1 ]; then
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_inertiaJ_${matlab_string_vec}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_MDPFIXB.m >> $zieldat
      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_MDPFIXB.m >> ${quelldat}.subsvar
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      # Die Massenmatrix wird als Dreiecksmatrix exportiert und hier wieder als symmetrische Matrix zusammengestellt.
      # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
      varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat | tr -d '[:space:]'`
      echo "%% Postprocessing: Reshape Output" >> $zieldat
      echo "% From vec2symmat_${robot_NQJ}_matlab.m" >> $zieldat
      sed "s/mv/$varname_tmp/g" $repo_pfad/codeexport/${robot_name}/tmp/vec2symmat_${robot_NQJ}_matlab.m >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar

    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi
  fi
  
  # Massenmatrix-Zeitableitung-Regressor
  quelldat=$repo_pfad/codeexport/${robot_name}/tmp/inertiaD_joint_joint_fixb_${maple_string_reg}_matlab.m
  zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_inertiaDJ_${matlab_string_reg}_slag_vp.m
  if [ -f $quelldat ]; then
    cat $head_pfad/robot_matlabtmp_inertiaDJ_${matlab_string_reg}.head.m > $zieldat
    printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
    source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
    printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
    printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
    sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi

  # Gravitationsmoment-Regressor
  quelldat=$repo_pfad/codeexport/${robot_name}/tmp/gravload_joint_fixb_${maple_string_reg}_matlab.m
  zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_gravloadJ_${matlab_string_reg}_slag_vp.m
  if [ -f $quelldat ]; then
    cat $head_pfad/robot_matlabtmp_gravloadJ_${matlab_string_reg}.head.m > $zieldat
    printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
    source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
    printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
    printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi
  
  # Gravitationsmoment-Vektor als Funktion der Dynamikparameter
  if [ $rm == 1 ]; then
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/gravload_joint_fixb_${maple_string_vec}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_gravloadJ_floatb_twist_${matlab_string_vec}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_gravloadJ_floatb_twist_${matlab_string_vec}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_MDPFIXB.m >> $zieldat
      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_MDPFIXB.m >> ${quelldat}.subsvar
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi
  fi
  
  # Inverse Dynamik-Regressor
  quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_joint_fixb_${maple_string_reg}_matlab.m
  zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdynJ_fixb_${matlab_string_reg}_slag_vp.m
  if [ -f $quelldat ]; then
    cat $head_pfad/robot_matlabtmp_invdynJ_fixb_${matlab_string_reg}.head.m > $zieldat
    printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
    source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJDD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
    printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJDD.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
    printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
    sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi
  
  
  # Inversdynamik-Vektor als Funktion der Dynamikparameter
  if [ $rm == 1 ]; then
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_joint_fixb_${maple_string_vec}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdynJ_fixb_${matlab_string_vec}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_invdynJ_fixb_${matlab_string_vec}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJDD.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_MDPFIXB.m >> $zieldat
      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJDD.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_MDPFIXB.m >> ${quelldat}.subsvar
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi
  fi

  # Kinetische Energie Regressor (Fixed Base)
  quelldat=$repo_pfad/codeexport/${robot_name}/tmp/energy_kinetic_fixb_${maple_string_reg}_matlab.m
  zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_energykin_fixb_${matlab_string_reg}_slag_vp.m
  if [ -f $quelldat ]; then
    cat $head_pfad/robot_matlabtmp_energykin_fixb_${matlab_string_reg}.head.m > $zieldat
    printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
    source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
    printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
    printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
    sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi


  # Potentielle Energie Regressor (Fixed base)
  quelldat=$repo_pfad/codeexport/${robot_name}/tmp/energy_potential_fixb_${maple_string_reg}_matlab.m
  zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_energypot_fixb_${matlab_string_reg}_slag_vp.m
  if [ -f $quelldat ]; then
    cat $head_pfad/robot_matlabtmp_energypot_fixb_${matlab_string_reg}.head.m > $zieldat
    printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
    source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
    printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
    printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
    sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi
done
