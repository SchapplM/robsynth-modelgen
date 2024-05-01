#!/bin/bash -e
# Generiere Matlab-Funktionen für Funktionen die auf der Berechnung des Impulses basieren
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-10
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover



echo "Generiere Matlabfunktionen: Impuls (benötigt für CMM und ZMP)"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
template_pfad=$repo_pfad/robot_codegen_scripts/templates_sym
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

basemethodenames=( twist eulangrpy )

for (( dynpar=1; dynpar<=2; dynpar++ ))
do
  # Funktionen für beide Basis-Modellierungen
  for basemeth in "${basemethodenames[@]}"
  do
    anglin_array=( lin ang ) # Schleife über Impuls und Drehimpuls (Funktionen sind sehr ähnlich)
    for anglin in "${anglin_array[@]}"
    do
      # Impuls
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/${anglin}momentum_worldframe_floatb_${basemeth}_par${dynpar}_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_momentum_${anglin}_floatb_${basemeth}_vp${dynpar}.m
      if [ -f $quelldat ]; then
        cat $head_pfad/robot_momentum_${anglin}_floatb_${basemeth}_vp${dynpar}.head.m > $zieldat
        printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
        if [ $anglin == "ang" ]; then
          cat $tmp_pfad/robot_matlabtmp_assert_rB.m >> $zieldat
        fi;
        if [ $basemeth == "twist" ]; then
          cat $tmp_pfad/robot_matlabtmp_assert_vB.m >> $zieldat
        else
          cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
          cat $tmp_pfad/robot_matlabtmp_assert_xDB.m >> $zieldat
        fi
        cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
        if [ $dynpar == 1 ]; then
          cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
          if [ $anglin == "ang" ]; then
            cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
          fi
        else
          cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
          if [ $anglin == "ang" ]; then
            cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
          fi
        fi
        printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
        if [ $anglin == "ang" ]; then
          cat $tmp_pfad/robot_matlabtmp_rB.m >> ${quelldat}.subsvar
        fi;
        if [ $basemeth == "twist" ]; then
          cat $tmp_pfad/robot_matlabtmp_vB.m >> ${quelldat}.subsvar
        else
          cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
          cat $tmp_pfad/robot_matlabtmp_xDB.m >> ${quelldat}.subsvar
        fi
        cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_m.m >> ${quelldat}.subsvar
        if [ $dynpar == 1 ]; then
          cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> ${quelldat}.subsvar
          if [ $anglin == "ang" ]; then
            cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> ${quelldat}.subsvar
          fi;
        else
          cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
          if [ $anglin == "ang" ]; then
            cat $tmp_pfad/robot_matlabtmp_par_If.m >> ${quelldat}.subsvar
          fi;
        fi
        printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
        sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
        cat $quelldat >> $zieldat
        source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 1 ${quelldat}.subsvar
      else
        echo "Code in ${quelldat##*/} nicht gefunden."
      fi

      # Impuls-Zeitableitung
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/${anglin}momentumD_worldframe_floatb_${basemeth}_par${dynpar}_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_momentumD_${anglin}_floatb_${basemeth}_vp${dynpar}.m
      if [ -f $quelldat ]; then
        cat $head_pfad/robot_momentumD_${anglin}_floatb_${basemeth}_vp${dynpar}.head.m > $zieldat
        printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_qJDD.m >> $zieldat
        if [ $anglin == "ang" ]; then
          cat $tmp_pfad/robot_matlabtmp_assert_rB.m >> $zieldat
        fi;
        if [ $basemeth == "twist" ]; then
          cat $tmp_pfad/robot_matlabtmp_assert_vB.m >> $zieldat
          cat $tmp_pfad/robot_matlabtmp_assert_aB.m >> $zieldat
        else
          cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
          cat $tmp_pfad/robot_matlabtmp_assert_xDB.m >> $zieldat
          cat $tmp_pfad/robot_matlabtmp_assert_xDDB.m >> $zieldat
        fi
        cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
        if [ $dynpar == 1 ]; then
          cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
          if [ $anglin == "ang" ]; then
            cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
          fi;
        else
          cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
          if [ $anglin == "ang" ]; then
            cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
          fi;
        fi
        printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_qJDD.m >> ${quelldat}.subsvar
        if [ $anglin == "ang" ]; then
          cat $tmp_pfad/robot_matlabtmp_rB.m >> ${quelldat}.subsvar
        fi;
        if [ $basemeth == "twist" ]; then
          cat $tmp_pfad/robot_matlabtmp_vB.m >> ${quelldat}.subsvar
          cat $tmp_pfad/robot_matlabtmp_aB.m >> ${quelldat}.subsvar
        else
          cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
          cat $tmp_pfad/robot_matlabtmp_xDB.m >> ${quelldat}.subsvar
          cat $tmp_pfad/robot_matlabtmp_xDDB.m >> ${quelldat}.subsvar
        fi
        cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_m.m >> ${quelldat}.subsvar
        if [ $dynpar == 1 ]; then
          cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> ${quelldat}.subsvar
          if [ $anglin == "ang" ]; then
            cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> ${quelldat}.subsvar
          fi;
        else
          cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
          if [ $anglin == "ang" ]; then
            cat $tmp_pfad/robot_matlabtmp_par_If.m >> ${quelldat}.subsvar
          fi;
        fi
        printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
        sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
        cat $quelldat >> $zieldat
        source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 1 ${quelldat}.subsvar
      else
        echo "Code in ${quelldat##*/} nicht gefunden."
      fi
    done


    # Centroidal Momentum Matrix
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/centroidal_momentum_matrix_worldframe_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_centr_momentum_matrix_floatb_${basemeth}_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_centr_momentum_matrix_floatb_${basemeth}_vp${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_rB.m >> $zieldat
      if [ $basemeth == "eulangrpy" ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
      fi
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_rB.m >> $zieldat
      if [ $basemeth == "eulangrpy" ]; then
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
       cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Time derivative of Centroidal Momentum Matrix
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/centroidal_momentum_matrix_worldframe_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_centr_momentum_matrixD_floatb_${basemeth}_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_centr_momentum_matrixD_floatb_${basemeth}_vp${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_rB.m >> $zieldat
        if [ $basemeth == "twist" ]; then
          cat $tmp_pfad/robot_matlabtmp_assert_vB.m >> $zieldat
        else
          cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
          cat $tmp_pfad/robot_matlabtmp_assert_xDB.m >> $zieldat
        fi
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
      fi
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qJD.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_rB.m >> $zieldat
        if [ $basemeth == "twist" ]; then
          cat $tmp_pfad/robot_matlabtmp_vB.m >> $zieldat
        else
          cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
          cat $tmp_pfad/robot_matlabtmp_xDB.m >> $zieldat
        fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
       cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi
  done # floatb_twist/floatb_eulxyz
done # par${dynpar}/par2

