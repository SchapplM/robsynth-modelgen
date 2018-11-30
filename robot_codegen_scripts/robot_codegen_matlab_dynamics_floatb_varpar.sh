#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Dynamik (explizit), floating base"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

basemethodenames=( twist eulxyz )

# Erstelle Matlab-Funktionen der explizit ausgerechneten Dynamik (nicht in Regressorform)

for (( dynpar=1; dynpar<=2; dynpar++ ))
do
  # Funktionen, die mit beiden Formulierungen der Basis-Darstellung korrekt sind
  for basemeth in "${basemethodenames[@]}"
  do

    # Gravitationsmoment (Gelenke)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/gravload_joint_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_gravloadJ_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_gravloadJ_floatb_${basemeth}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
      fi

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
      fi
      cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> ${quelldat}.subsvar
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
      fi

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 1 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Kinetische Energie (Floating Base)
    if [ $dynpar == 1 ]; then
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/energy_kinetic_floatb_${basemeth}_worldframe_par${dynpar}_matlab.m
    else
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/energy_kinetic_floatb_${basemeth}_linkframe_par${dynpar}_matlab.m
    fi
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_energykin_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_energykin_floatb_${basemeth}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
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

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar

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
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> ${quelldat}.subsvar
      fi

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Potentielle Energie (Floating base)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/energy_potential_floatb_${basemeth}_worldframe_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_energypot_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_energypot_floatb_${basemeth}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_rB.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
      fi

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_rB.m >> ${quelldat}.subsvar
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
      fi
      cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> ${quelldat}.subsvar
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
      fi

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi
  done


  # Funktionen, die nur mit der Euler-Winkeldarstellung für die Basis korrekt sind
  for basemeth in "eulxyz"
  do
    # Gravitationsmoment (Basis)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/gravload_base_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_gravloadB_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_gravloadB_floatb_${basemeth}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
      fi

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
      fi
      cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> ${quelldat}.subsvar
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
      fi

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 1 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Coriolisvektor (Floating Base, Basis)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/coriolisvec_base_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_coriolisvecB_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_coriolisvecB_floatb_${basemeth}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
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

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
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
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> ${quelldat}.subsvar
      fi

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 1 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Coriolisvektor (Floating Base, Gelenke)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/coriolisvec_joint_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_coriolisvecJ_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_coriolisvecJ_floatb_${basemeth}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
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

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
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
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> ${quelldat}.subsvar
      fi

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 1 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Coriolisvektor (Floating Base, Gesamt: Basis und Gelenke)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/coriolisvec_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_coriolisvec_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_coriolisvec_floatb_${basemeth}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
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

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
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
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> ${quelldat}.subsvar
      fi

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 1 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Coriolismatrix (Floating Base: Einfluss auf Gelenke)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/coriolismat_joint_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_coriolismatJ_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_coriolismatJ_floatb_${basemeth}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
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

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
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
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> ${quelldat}.subsvar
      fi

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Coriolismatrix (Floating Base: Gesamt)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/coriolismat_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_coriolismat_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_coriolismat_floatb_${basemeth}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
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

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
      if [ $basemeth == "twist" ]; then
        cat $tmp_pfad/robot_matlabtmp_vB.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >>${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_xDB.m >> ${quelldat}.subsvar
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> ${quelldat}.subsvar
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> ${quelldat}.subsvar
      fi

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Massenmatrix (Gesamt)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/inertia_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_inertia_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_inertia_floatb_${basemeth}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
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

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> ${quelldat}.subsvar
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> ${quelldat}.subsvar
      fi

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
      varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
      echo "%% Postprocessing: Reshape Output" >> $zieldat
      echo "% From vec2symmat_$((6+robot_NQJ))_matlab.m" >> $zieldat
      sed "s/mv/$varname_tmp/g" $repo_pfad/codeexport/${robot_name}/tmp/vec2symmat_$((6+robot_NQJ))_matlab.m >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Massenmatrix (Basis-Gelenke)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/inertia_joint_base_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_inertiaJB_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_inertiaJB_floatb_${basemeth}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
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

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> ${quelldat}.subsvar
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> ${quelldat}.subsvar
      fi

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Massenmatrix (Basis-Basis)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/inertia_base_base_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_inertiaB_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_inertiaB_floatb_${basemeth}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
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

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> ${quelldat}.subsvar
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> ${quelldat}.subsvar
      fi

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
      varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
      echo "%% Postprocessing: Reshape Output" >> $zieldat
      echo "% From vec2symmat_6_matlab.m" >> $zieldat
      sed "s/mv/$varname_tmp/g" $repo_pfad/codeexport/${robot_name}/tmp/vec2symmat_6_matlab.m >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Massenmatrix-Zeitableitung (Floating Base: Gesamt)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/inertia_time_derivative_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_inertiaD_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_inertiaD_floatb_${basemeth}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
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

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_xDB.m >> ${quelldat}.subsvar
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> ${quelldat}.subsvar
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> ${quelldat}.subsvar
      fi

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
      varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
      echo "%% Postprocessing: Reshape Output" >> $zieldat
      echo "% From vec2symmat_$((6+robot_NQJ))_matlab.m" >> $zieldat
      sed "s/mv/$varname_tmp/g" $repo_pfad/codeexport/${robot_name}/tmp/vec2symmat_$((6+robot_NQJ))_matlab.m >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Inverse Dynamik (Gelenke, Floating Base)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_joint_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdynJ_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_invdynJ_floatb_${basemeth}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJDD.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_vB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_aB.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_xDB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_xDDB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
      fi

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJDD.m >> ${quelldat}.subsvar
      if [ $basemeth == "twist" ]; then
        cat $tmp_pfad/robot_matlabtmp_vB.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_aB.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_xDB.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_xDDB.m >> ${quelldat}.subsvar
      fi
      cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> ${quelldat}.subsvar
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> ${quelldat}.subsvar
      fi

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 1 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Inverse Dynamik (Basis, Floating Base)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_base_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdynB_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_invdynB_floatb_${basemeth}_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJDD.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_vB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_aB.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_xDB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_xDDB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
      fi

      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJDD.m >> ${quelldat}.subsvar
      if [ $basemeth == "twist" ]; then
        cat $tmp_pfad/robot_matlabtmp_vB.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_aB.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_xDB.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_xDDB.m >> ${quelldat}.subsvar
      fi
      cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> ${quelldat}.subsvar
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> ${quelldat}.subsvar
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> ${quelldat}.subsvar
      fi

      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 1 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

  done # floatb_twist/floatb_eulxyz
done # par1/par2
