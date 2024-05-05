#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur parameterlinearen Dynamik und wird von robot_codegen_matlab_varpar_par.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Tim Job (Studienarbeit bei Moritz Schappler), 2018-12
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover

echo "Generiere Matlabfunktionen: Parameterlineare Dynamik PKM"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
template_pfad=$repo_pfad/robot_codegen_scripts/templates_sym
# Initialisiere Variablen
source robot_codegen_tmpvar_bash_par.sh
source $repo_pfad/robot_codegen_definitions/robot_env_par.sh

# Minimalparametervektor (Fixed Base)
quelldat=$repo_pfad/codeexport/${robot_name}/tmp/minimal_parameter_parrob_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_minimal_parameter_para.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_convert_par2_MPV_para.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
  source $repo_pfad/scripts/set_inputdim_line_par.sh $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_m_parallel.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_mrcom_parallel.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_If_parallel.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m >> $zieldat
  # echo "%% Conversion Parameterset 1 -> Parameterset 2" >> $zieldat
  # echo "[mrcges, Ifges] = inertial_parameters_convert_par1_par2(rSges, Icges, m);" >> $zieldat

  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_m_parallel.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_mrcom_parallel.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_If_parallel.m >> ${quelldat}.subsvar

  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 1 0 ${quelldat}.subsvar
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
  # Inverse Dynamik
  coordmaple=( actcoord plfcoord)
  coordmatlab=( qa pf )
  for (( coord=0; coord<=1; coord++ )); do # 0=act joints, 1=platform
    # Zeichenkette für die Koordinatensysteme, für die die Dynamik-Terme definiert sind.
    costrmpl=${coordmaple[$coord]}
    costrmat=${coordmatlab[$coord]}
    
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_para_${costrmpl}_${maple_string_reg}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdyn_para_${costrmat}_${matlab_string_reg}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_invdynJ_para_${costrmat}_${matlab_string_reg}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line_par.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xDP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xDDP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_legFrame_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m >> $zieldat

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ_parallel.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_xP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_xDP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_xDDP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

      cat $tmp_pfad/robot_matlabtmp_legFrame_parallel.m >> ${quelldat}.subsvar

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden. "
    fi
    
    # Inverse Dynamik mit MPV bereits eingesetzt
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_para_${costrmpl}_reg_mdp_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdyn_para_${costrmat}_mdp.m
    if [ $rm == 1 ]; then
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_invdynJ_para_${costrmat}_mdp.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line_par.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xDP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xDDP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_legFrame_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_MDPFIXB_parallel.m >> $zieldat

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ_parallel.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_xP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_xDP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_xDDP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_MDPFIXB_parallel.m >> ${quelldat}.subsvar

      cat $tmp_pfad/robot_matlabtmp_legFrame_parallel.m >> ${quelldat}.subsvar

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden. "
    fi
    fi
  done

  # Gravitationsvektor
  coordmaple=( actcoord plfcoord)
  coordmatlab=( qa pf )
  for (( coord=0; coord<=1; coord++ )); do # 0=act joints, 1=platform
    # Zeichenkette für die Koordinatensysteme, für die die Dynamik-Terme definiert sind.
    costrmpl=${coordmaple[$coord]}
    costrmat=${coordmatlab[$coord]}
    
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_para_${costrmpl}_taug_${maple_string_reg}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_gravload_para_${costrmat}_${matlab_string_reg}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_gravloadJ_para_${costrmat}_${matlab_string_reg}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line_par.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_legFrame_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m >> $zieldat

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ_parallel.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_xP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

      cat $tmp_pfad/robot_matlabtmp_legFrame_parallel.m >> ${quelldat}.subsvar

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden. "
    fi
    
    # Gravitationsvektor mit MPV bereits eingesetzt
    if [ $rm == 1 ]; then
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_para_${costrmpl}_taugreg_mdp_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_gravload_para_${costrmat}_mdp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_gravloadJ_para_${costrmat}_mdp.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line_par.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_legFrame_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_MDPFIXB_parallel.m >> $zieldat

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ_parallel.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_xP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_MDPFIXB_parallel.m >> ${quelldat}.subsvar

      cat $tmp_pfad/robot_matlabtmp_legFrame_parallel.m >> ${quelldat}.subsvar

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden. "
    fi
    fi
  done

  # Coriolisvektor
  coordmaple=( actcoord plfcoord)
  coordmatlab=( qa pf )
  for (( coord=0; coord<=1; coord++ )); do # 0=act joints, 1=platform
    # Zeichenkette für die Koordinatensysteme, für die die Dynamik-Terme definiert sind.
    costrmpl=${coordmaple[$coord]}
    costrmat=${coordmatlab[$coord]}
    
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_para_${costrmpl}_tauC_${maple_string_reg}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_coriolisvec_para_${costrmat}_${matlab_string_reg}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_coriolisvecJ_para_${costrmat}_${matlab_string_reg}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line_par.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xDP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_legFrame_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m >> $zieldat

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ_parallel.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_xP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_xDP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

      cat $tmp_pfad/robot_matlabtmp_legFrame_parallel.m >> ${quelldat}.subsvar

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden. "
    fi
    
    # Coriolisvektor mit MPV bereits eingesetzt
    if [ $rm == 1 ]; then
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_para_${costrmpl}_tauCreg_mdp_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_coriolisvec_para_${costrmat}_mdp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_coriolisvecJ_para_${costrmat}_mdp.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line_par.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xDP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_legFrame_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_MDPFIXB_parallel.m >> $zieldat

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ_parallel.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_xP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_xDP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_MDPFIXB_parallel.m >> ${quelldat}.subsvar

      cat $tmp_pfad/robot_matlabtmp_legFrame_parallel.m >> ${quelldat}.subsvar

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden. "
    fi
    fi
  done

  # Massenmatrix
  coordmaple=( actcoord plfcoord)
  coordmatlab=( qa pf )
  for (( coord=0; coord<=1; coord++ )); do # 0=act joints, 1=platform
    # Zeichenkette für die Koordinatensysteme, für die die Dynamik-Terme definiert sind.
    costrmpl=${coordmaple[$coord]}
    costrmat=${coordmatlab[$coord]}
    
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_para_${costrmpl}_MM_${maple_string_reg}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_inertia_para_${costrmat}_${matlab_string_reg}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_inertiaJ_para_${costrmat}_${matlab_string_reg}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line_par.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_legFrame_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m >> $zieldat

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
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
      echo "Code in ${quelldat##*/} nicht gefunden. "
    fi
    
    # Massenmatrix mit MPV bereits eingesetzt
    if [ $rm == 1 ]; then
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_para_${costrmpl}_MMreg_mdp_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_inertia_para_${costrmat}_mdp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_inertiaJ_para_${costrmat}_mdp.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line_par.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_legFrame_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_MDPFIXB_parallel.m >> $zieldat

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ_parallel.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_xP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_MDPFIXB_parallel.m >> ${quelldat}.subsvar

      cat $tmp_pfad/robot_matlabtmp_legFrame_parallel.m >> ${quelldat}.subsvar

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      # Die Massenmatrix wird als Dreiecksmatrix exportiert und hier wieder als symmetrische Matrix zusammengestellt.
      # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
      varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat | tr -d '[:space:]'`
      echo "%% Postprocessing: Reshape Output" >> $zieldat
      echo "% From vec2mat_${parallel_NX}_matlab.m" >> $zieldat
      sed "s/mv/$varname_tmp/g" $repo_pfad/codeexport/${robot_name}/tmp/vec2mat_${parallel_NX}_matlab.m >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden. "
    fi
    fi
  done
done # for rm (regressor_mode)
