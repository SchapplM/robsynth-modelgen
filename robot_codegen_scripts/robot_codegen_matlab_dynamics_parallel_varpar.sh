#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Dynamik PKM"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
template_pfad=$repo_pfad/robot_codegen_scripts/templates_sym
# Initialisiere Variablen
#source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Inverse Kinematik
quelldat=$repo_pfad/codeexport/${robot_name}/tmp/Jinv_para_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_Jinv.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_jacobia_parallel.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
  source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
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
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Erstelle Matlab-Funktionen der explizit ausgerechneten Dynamik (nicht in Regressorform)
for (( dynpar=1; dynpar<=2; dynpar++ ))
do
	# Gravitationsvektor
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/gravvec_para_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_gravload_para_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_gravloadJ_para_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
	  cat $tmp_pfad/robot_matlabtmp_assert_xP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_m_parallel.m >> $zieldat
	  cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_rcom_parallel.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_mrcom_parallel.m >> $zieldat
      fi
	  cat $tmp_pfad/robot_matlabtmp_assert_legFrame_parallel.m >> $zieldat
	  cat $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m >> $zieldat
      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ_parallel.m >> ${quelldat}.subsvar
	  cat $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m >> ${quelldat}.subsvar
	  cat $tmp_pfad/robot_matlabtmp_xP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_m_parallel.m >> ${quelldat}.subsvar
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom_parallel.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom_parallel.m >> ${quelldat}.subsvar
      fi
	  cat $tmp_pfad/robot_matlabtmp_legFrame_parallel.m >> ${quelldat}.subsvar
      
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi
	
	
    # Coriolisvektor
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/coriolisvec_para_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_coriolisvec_para_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_coriolisvecJ_para_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
	  cat $tmp_pfad/robot_matlabtmp_assert_xP.m >> $zieldat
	  cat $tmp_pfad/robot_matlabtmp_assert_xPD.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_m_parallel.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_rcom_parallel.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_Ic_parallel.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_mrcom_parallel.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_If_parallel.m >> $zieldat
      fi
	  cat $tmp_pfad/robot_matlabtmp_assert_legFrame_parallel.m >> $zieldat
	  cat $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m >> $zieldat
      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ_parallel.m >> ${quelldat}.subsvar
	  cat $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m >> ${quelldat}.subsvar
	  cat $tmp_pfad/robot_matlabtmp_xP.m >> ${quelldat}.subsvar
	  cat $tmp_pfad/robot_matlabtmp_xPD.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_m_parallel.m >> ${quelldat}.subsvar
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom_parallel.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom_parallel.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_If_parallel.m >> ${quelldat}.subsvar
      fi
	  cat $tmp_pfad/robot_matlabtmp_legFrame_parallel.m >> ${quelldat}.subsvar
      
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi
	
	
	# Massenmatrix
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/inertia_para_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_inertia_para_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_inertiaJ_para_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
	  cat $tmp_pfad/robot_matlabtmp_assert_xP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_m_parallel.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_rcom_parallel.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_Ic_parallel.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_mrcom_parallel.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_If_parallel.m >> $zieldat
      fi
	  cat $tmp_pfad/robot_matlabtmp_assert_legFrame_parallel.m >> $zieldat
	  cat $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m >> $zieldat
      
      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ_parallel.m >> ${quelldat}.subsvar
	  cat $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m >> ${quelldat}.subsvar
	  cat $tmp_pfad/robot_matlabtmp_xP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_m_parallel.m >> ${quelldat}.subsvar
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom_parallel.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom_parallel.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_If_parallel.m >> ${quelldat}.subsvar
      fi
	  cat $tmp_pfad/robot_matlabtmp_legFrame_parallel.m >> ${quelldat}.subsvar
      
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
      #varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat | tr -d '[:space:]'`
      #echo "%% Postprocessing: Reshape Output" >> $zieldat
      #echo "% From vec2symmat_${robot_NQJ}_matlab.m" >> $zieldat
      #sed "s/mv/$varname_tmp/g" $repo_pfad/codeexport/${robot_name}/tmp/vec2symmat_${robot_NQJ}_matlab.m >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

	
    # Inverse Dynamik
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_para_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdyn_para_slag_vp${dynpar}.m
    cat $head_pfad/robot_matlabtmp_invdynJ_para_par${dynpar}.head.m > $zieldat
    printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
    #source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
	cat $tmp_pfad/robot_matlabtmp_assert_xP.m >> $zieldat
	cat $tmp_pfad/robot_matlabtmp_assert_xPD.m >> $zieldat
	cat $tmp_pfad/robot_matlabtmp_assert_xPDD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJ_parallel.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m_parallel.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_assert_rcom_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_Ic_parallel.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_assert_mrcom_parallel.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_If_parallel.m >> $zieldat
    fi
	cat $tmp_pfad/robot_matlabtmp_assert_legFrame_parallel.m >> $zieldat
	cat $tmp_pfad/robot_matlabtmp_assert_koppelP_parallel.m >> $zieldat
    if [ -f $quelldat ]; then
      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ_parallel.m >> ${quelldat}.subsvar
	  cat $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m >> ${quelldat}.subsvar
	  cat $tmp_pfad/robot_matlabtmp_xP.m >> ${quelldat}.subsvar
	  cat $tmp_pfad/robot_matlabtmp_xPD.m >> ${quelldat}.subsvar
	  cat $tmp_pfad/robot_matlabtmp_xPDD.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_m_parallel.m >> ${quelldat}.subsvar
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom_parallel.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom_parallel.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_If_parallel.m >> ${quelldat}.subsvar
      fi
	  cat $tmp_pfad/robot_matlabtmp_legFrame_parallel.m >> ${quelldat}.subsvar
	  
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
    else
      echo "Code in ${quelldat##*/} nicht gefunden. "
    fi
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar

done # par1/par2
