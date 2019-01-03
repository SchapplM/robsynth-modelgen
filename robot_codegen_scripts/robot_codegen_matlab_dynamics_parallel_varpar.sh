#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur PKM-Dynamik und wird von robot_codegen_matlab_varpar_par.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Tim Job (Studienarbeit bei Moritz Schappler), 2018-12
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover

echo "Generiere Matlabfunktionen: Dynamik PKM"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
template_pfad=$repo_pfad/robot_codegen_scripts/templates_par
# Initialisiere Variablen
source robot_codegen_tmpvar_bash_par.sh
source $repo_pfad/robot_codegen_definitions/robot_env_par.sh

# Definitionen für Namen der zu ladenden Dateien
coordmaple=( actcoord plfcoord invalid )
coordmatlab=( qa pf qa )

# Erstelle Matlab-Funktionen der explizit ausgerechneten Dynamik (nicht in Regressorform)
# Generiere die verschiedenen Varianten mit zwei geschachtelten Schleifen, um den Code in dieser Datei übersichtlicher zu hatel
for (( dynpar=1; dynpar<=2; dynpar++ )); do
  # Schleife 1: Dynamikparameter
  for (( coord=0; coord<=2; coord++ )); do # 0=act joints, 1=platform, 2=act joints (numerische Berechnung)
    # Schleife 2: Koordinaten der Dynamikterme
    # 0: Antriebskoordinaten mit symbolischer Berechnung (symbolischer Invertierung der inversen Jacobi)
    # 1: Plattformkoordinaten, symbolische Berechnung
    # 2: Wie 0 in Antriebskoordinaten, aber mit numerischer Invertierung der inversen Jacobi.
    
    # Zeichenkette für die Koordinatensysteme, für die die Dynamik-Terme definiert sind.
    costrmpl=${coordmaple[$coord]}
    costrmat=${coordmatlab[$coord]}
    
    # Gravitationsvektor
    if (( $coord < 2 )); then
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/gravvec_para_${costrmpl}_par${dynpar}_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_gravload_para_${costrmat}_slag_vp${dynpar}.m
    else
      quelldat="/dummy" # noch nicht implementiert
      zieldat="/dummy"
    fi
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_gravloadJ_para_${costrmat}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line_par.sh $zieldat
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
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 1 0 ${quelldat}.subsvar
    elif  (( $coord < 2 )); then
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Coriolisvektor
    if (( $coord < 2 )); then
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/coriolisvec_para_${costrmpl}_par${dynpar}_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_coriolisvec_para_${costrmat}_slag_vp${dynpar}.m
    else
      quelldat="/dummy" # noch nicht implementiert
      zieldat="/dummy"
    fi
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_coriolisvecJ_para_${costrmat}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line_par.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xDP.m >> $zieldat
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
      cat $tmp_pfad/robot_matlabtmp_xDP.m >> ${quelldat}.subsvar
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
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 1 0 ${quelldat}.subsvar
    elif  (( $coord < 2 )); then
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Massenmatrix
    if (( $coord < 2 )); then
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/inertia_para_${costrmpl}_par${dynpar}_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_inertia_para_${costrmat}_slag_vp${dynpar}.m
    else
      quelldat="/dummy" # noch nicht implementiert
      zieldat="/dummy"
    fi
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_inertiaJ_para_${costrmat}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line_par.sh $zieldat
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
      # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess_par.sh)
      #varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat | tr -d '[:space:]'`
      #echo "%% Postprocessing: Reshape Output" >> $zieldat
      #echo "% From vec2symmat_${robot_NQJ}_matlab.m" >> $zieldat
      #sed "s/mv/$varname_tmp/g" $repo_pfad/codeexport/${robot_name}/tmp/vec2symmat_${robot_NQJ}_matlab.m >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 1 0 ${quelldat}.subsvar
    elif  (( $coord < 2 )); then
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Inverse Dynamik
    if (( $coord < 2 )); then
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_para_${costrmpl}_par${dynpar}_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdyn_para_${costrmat}_slag_vp${dynpar}.m
    else
      quelldat=$repo_pfad/robot_codegen_scripts/templates_par/robot_matlabtmp_invdynJ_para_qa_par1.m.body
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdyn_para_${costrmat}_slagn_vp${dynpar}.m
    fi
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_invdynJ_para_${costrmat}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line_par.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xDP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_xDDP.m >> $zieldat
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

      if (($coord < 2)); then
        printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_qJ_parallel.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_koppelP_parallel.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_xP.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_xDP.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_xDDP.m >> ${quelldat}.subsvar
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
      fi
      
      if (( $coord < 2 )); then
        printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
        sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
        cat $quelldat >> $zieldat
      else
        cat ${template_pfad}/robot_matlabtmp_invdynJ_para_qa_par${dynpar}.m.body >> $zieldat
      fi
      source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden. "
    fi
  done # coord: pf/qa/qa_num
done # par1/par2
