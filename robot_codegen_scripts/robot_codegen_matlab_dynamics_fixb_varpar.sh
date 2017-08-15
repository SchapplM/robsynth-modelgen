#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Dynamik (explizit), fixed-base"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/robot_codegen_scripts/tmp
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Schleife für beide Basis-Darstellungen durchgehen.
# Die Methode "twist" muss am Ende kommen, damit Basis-unabhängige Funktionen mit den Ergebnissen dieser Methode
# generiert werden ("twist"-Code ist kürzer; Zuerst erstellte Datei mit "eulangrpy" wird dann überschrieben).
basemethodenames=( eulangrpy twist )

# Erstelle Matlab-Funktionen der explizit ausgerechneten Dynamik (nicht in Regressorform)
for (( dynpar=1; dynpar<=2; dynpar++ ))
do
  for basemeth in "${basemethodenames[@]}"
  do

    # Coriolisvektor (Fixed Base)
    quelldat=$repo_pfad/codeexport/${robot_name}/coriolisvec_joint_fixb_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_coriolisvecJ_fixb_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_coriolisvecJ_fixb_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
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
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
      fi
      
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 1
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Coriolismatrix (Fixed Base)
    quelldat=$repo_pfad/codeexport/${robot_name}/coriolismat_joint_fixb_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_coriolismatJ_fixb_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_coriolismatJ_fixb_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
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
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
      fi
      
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Massenmatrix (Gelenke)
    quelldat=$repo_pfad/codeexport/${robot_name}/inertia_joint_joint_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_inertiaJ_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_inertiaJ_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
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
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
      fi
      
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
      varname_tmp=`grep "=" $zieldat | tail -1 | sed 's/\(.*\)=.*/\1/' | tr -d '[:space:]'`
      echo "%% Postprocessing: Reshape Output" >> $zieldat
      echo "% From vec2symmat_${robot_NQJ}_matlab.m" >> $zieldat
      sed "s/mv/$varname_tmp/g" $repo_pfad/codeexport/${robot_name}/vec2symmat_${robot_NQJ}_matlab.m >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Kinetische Energie (Fixed Base)
    if [ $dynpar == 1 ]; then
      quelldat=$repo_pfad/codeexport/${robot_name}/energy_kinetic_fixb_worldframe_par${dynpar}_matlab.m
    else
      quelldat=$repo_pfad/codeexport/${robot_name}/energy_kinetic_fixb_linkframe_par${dynpar}_matlab.m
    fi
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_energykin_fixb_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_energykin_fixb_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
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
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
      fi
      
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Massenmatrix-Zeitableitung (Gelenke)
    quelldat=$repo_pfad/codeexport/${robot_name}/inertia_joint_joint_time_derivative_floatb_${basemeth}_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_inertiaDJ_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_inertiaDJ_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
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
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
      fi
      
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
      varname_tmp=`grep "=" $zieldat| tail -1 | sed 's/\(.*\)=.*/\1/' | tr -d '[:space:]'`
      echo "%% Postprocessing: Reshape Output" >> $zieldat
      echo "% From vec2symmat_${robot_NQJ}_matlab.m" >> $zieldat
      sed "s/mv/$varname_tmp/g" $repo_pfad/codeexport/${robot_name}/vec2symmat_${robot_NQJ}_matlab.m >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Potentielle Energie (Fixed base)
    quelldat=$repo_pfad/codeexport/${robot_name}/energy_potential_fixb_worldframe_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_energypot_fixb_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_energypot_fixb_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
      fi
      
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> $zieldat
      printf "rxs_base=0;\nrys_base=0;\nrzs_base=0;\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
      printf "M0=0;%%Masse der Basis nicht relevant.\n" >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
        printf "SX0=0;SY0=0;SZ0=0;%%Basis-Schwerpunkt nicht relevant.\n" >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
        printf "MX0=0;MY0=0;MZ0=0;%%Basis-1. Moment nicht relevant.\n" >> $zieldat
      fi
      
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Inverse Dynamik (Fixed Base)
    quelldat=$repo_pfad/codeexport/${robot_name}/invdyn_fixb_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_invdynJ_fixb_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_invdynJ_fixb_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qJDD.m >> $zieldat
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
      
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qJD.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qJDD.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
      if [ $dynpar == 1 ]; then
        cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
      fi
      
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi
  done # floatb_twist/floatb_eulangrpy
done # par1/par2
