#!/bin/bash
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/robot_codegen_scripts/tmp/
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Erstelle Matlab-Funktionen der explizit ausgerechneten Dynamik (nicht in Regressorform)
for (( dynpar=1; dynpar<=2; dynpar++ ))
do
  # Gravitationsmoment
  quelldat=$repo_pfad/codeexport/${robot_name}_joint_gravload_par${dynpar}_matlab.m
  zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_gravload_joint_sym_lag_varpar_par${dynpar}.m
  if [ -f $quelldat ]; then
    cat $tmp_pfad/robot_matlabtmp_gravload_joint_par${dynpar}.head.m > $zieldat
    printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
    fi
    printf "\n%%%% Variable Initialization" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_g.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
    fi
    printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi

  # Coriolisvektor (Floating Base)
  quelldat=$repo_pfad/codeexport/${robot_name}_coriolisvec_joint_floatb_par${dynpar}_matlab.m
  zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_coriolisvec_joint_floatb_sym_lag_varpar_par${dynpar}.m
  if [ -f $quelldat ]; then
    cat $tmp_pfad/robot_matlabtmp_coriolisvec_joint_floatb_par${dynpar}.head.m > $zieldat
    printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_vB.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
    fi
    printf "\n%%%% Variable Initialization" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_vB.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
    fi
    printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi

  # Coriolisvektor (Fixed Base)
  quelldat=$repo_pfad/codeexport/${robot_name}_coriolisvec_joint_fixb_par${dynpar}_matlab.m
  zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_coriolisvec_joint_fixb_sym_lag_varpar_par${dynpar}.m
  if [ -f $quelldat ]; then
    cat $tmp_pfad/robot_matlabtmp_coriolisvec_joint_fixb_par${dynpar}.head.m > $zieldat
    printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
    fi
    printf "\n%%%% Variable Initialization" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
    fi
    printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi


  # Coriolismatrix (Floating Base)
  quelldat=$repo_pfad/codeexport/${robot_name}_coriolismat_joint_floatb_par${dynpar}_matlab.m
  zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_coriolismat_joint_floatb_sym_lag_varpar_par${dynpar}.m
  if [ -f $quelldat ]; then
    cat $tmp_pfad/robot_matlabtmp_coriolismat_joint_floatb_par${dynpar}.head.m > $zieldat
    printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_vB.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
    fi
    printf "\n%%%% Variable Initialization" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_vB.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
    fi
    printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi

  # Coriolismatrix (Fixed Base)
  quelldat=$repo_pfad/codeexport/${robot_name}_coriolismat_joint_fixb_par${dynpar}_matlab.m
  zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_coriolismat_joint_fixb_sym_lag_varpar_par${dynpar}.m
  if [ -f $quelldat ]; then
    cat $tmp_pfad/robot_matlabtmp_coriolismat_joint_fixb_par${dynpar}.head.m > $zieldat
    printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
    fi
    printf "\n%%%% Variable Initialization" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
    fi
    printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi

  # Massenmatrix (Gelenke)
  quelldat=$repo_pfad/codeexport/${robot_name}_inertia_joint_joint_par${dynpar}_matlab.m
  zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_inertia_joint_sym_lag_varpar_par${dynpar}.m
  if [ -f $quelldat ]; then
    cat $tmp_pfad/robot_matlabtmp_inertia_joint_par${dynpar}.head.m > $zieldat
    printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
    fi
    printf "\n%%%% Variable Initialization" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
    fi
    printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi

  # Massenmatrix (Basis-Gelenke)
  quelldat=$repo_pfad/codeexport/${robot_name}_inertia_joint_base_par${dynpar}_matlab.m
  zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_inertia_joint_base_sym_lag_varpar_par${dynpar}.m
  if [ -f $quelldat ]; then
    cat $tmp_pfad/robot_matlabtmp_inertia_joint_base_par${dynpar}.head.m > $zieldat
    printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
    fi
    printf "\n%%%% Variable Initialization" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
    fi
    printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi


  # Massenmatrix-Zeitableitung (Gelenke)
  quelldat=$repo_pfad/codeexport/${robot_name}_inertia_joint_joint_time_derivative_par${dynpar}_matlab.m
  zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_inertiaD_joint_sym_lag_varpar_par${dynpar}.m
  if [ -f $quelldat ]; then
    cat $tmp_pfad/robot_matlabtmp_inertiaD_joint_par${dynpar}.head.m > $zieldat
    printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
    fi
    printf "\n%%%% Variable Initialization" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
    fi
    printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi

  # Kinetische Energie (Floating Base)
  if [ $dynpar == 1 ]; then
    quelldat=$repo_pfad/codeexport/${robot_name}_energy_kinetic_floatb_baseframe_par${dynpar}_matlab.m
  else
    quelldat=$repo_pfad/codeexport/${robot_name}_energy_kinetic_floatb_linkframe_par${dynpar}_matlab.m
  fi
  zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_energykin_floatb_sym_lag_varpar_par${dynpar}.m
  if [ -f $quelldat ]; then
    cat $tmp_pfad/robot_matlabtmp_energykin_floatb_par${dynpar}.head.m > $zieldat
    printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_vB.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
    fi
    printf "\n%%%% Variable Initialization" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_vB.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
    fi
    printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi

  # Kinetische Energie (Fixed Base)
  if [ $dynpar == 1 ]; then
    quelldat=$repo_pfad/codeexport/${robot_name}_energy_kinetic_fixb_baseframe_par${dynpar}_matlab.m
  else
    quelldat=$repo_pfad/codeexport/${robot_name}_energy_kinetic_fixb_linkframe_par${dynpar}_matlab.m
  fi
  zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_energykin_fixb_sym_lag_varpar_par${dynpar}.m
  if [ -f $quelldat ]; then
    cat $tmp_pfad/robot_matlabtmp_energykin_fixb_par${dynpar}.head.m > $zieldat
    printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
    fi
    printf "\n%%%% Variable Initialization" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
    fi
    printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi

  # Potentielle Energie (Floating base)
  quelldat=$repo_pfad/codeexport/${robot_name}_energy_potential_baseframe_par${dynpar}_matlab.m
  zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_energypot_floatb_sym_lag_varpar_par${dynpar}.m
  if [ -f $quelldat ]; then
    cat $tmp_pfad/robot_matlabtmp_energypot_floatb_par${dynpar}.head.m > $zieldat
    printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_rB.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
    fi
    printf "\n%%%% Variable Initialization" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_rB.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_g.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
    fi
    printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi

  # Potentielle Energie (Fixed base)
  quelldat=$repo_pfad/codeexport/${robot_name}_energy_potential_baseframe_par${dynpar}_matlab.m
  zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_energypot_fixb_sym_lag_varpar_par${dynpar}.m
  if [ -f $quelldat ]; then
    cat $tmp_pfad/robot_matlabtmp_energypot_fixb_par${dynpar}.head.m > $zieldat
    printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
    fi
    printf "\n%%%% Variable Initialization" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
    printf "rxs_base=0;\nrys_base=0;\nrzs_base=0;\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_g.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
    printf "M1=0;%%Masse der Basis nicht relevant.\n" >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
      printf "SX1=0;SY1=0;SZ1=0;%%nicht relevant.\n" >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
      printf "MX1=0;MY1=0;MZ1=0;%%nicht relevant.\n" >> $zieldat
    fi
    printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi

  # Inverse Dynamik (Floating Base)
  quelldat=$repo_pfad/codeexport/${robot_name}_invdyn_joint_floatb_par${dynpar}_matlab.m
  zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_invdyn_joint_floatb_sym_lag_varpar_par${dynpar}.m
  if [ -f $quelldat ]; then
    cat $tmp_pfad/robot_matlabtmp_invdyn_joint_floatb_par${dynpar}.head.m > $zieldat
    printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qDD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_vB.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_aB.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
    fi
    printf "\n%%%% Variable Initialization" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_qDD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_vB.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_aB.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_g.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
    fi
    printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi


  # Inverse Dynamik (Fixed Base)
  quelldat=$repo_pfad/codeexport/${robot_name}_invdyn_fixb_par${dynpar}_matlab.m
  zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_invdyn_fixb_sym_lag_varpar_par${dynpar}.m
  if [ -f $quelldat ]; then
    cat $tmp_pfad/robot_matlabtmp_invdyn_fixb_par${dynpar}.head.m > $zieldat
    printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qDD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
    fi
    printf "\n%%%% Variable Initialization" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_qDD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_g.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
    fi
    printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi

done
