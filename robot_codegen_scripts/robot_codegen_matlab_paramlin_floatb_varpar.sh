#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur parameterlinearen Form der Floating-Base-Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2017-02
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Dynamik-Regressor (Floating Base)"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/robot_codegen_scripts/tmp
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

basemethodenames=( eulangrpy )

# Erstelle Matlab-Funktionen der Regressorform für Floating Base Modell
for basemeth in "${basemethodenames[@]}"
do
  # Minimalparametervektor (Floating Base)
  quelldat=$repo_pfad/codeexport/${robot_name}/minimal_parameter_vector_floatb_${basemeth}_matlab.m
  zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_convert_par2_MPV_floatb_${basemeth}.m
  if [ -f $quelldat ]; then
    cat ${tmp_pfad}_head/robot_matlabtmp_convert_par2_MPV_floatb_${basemeth}.head.m > $zieldat
    printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
    # echo "%% Conversion Parameterset 1 -> Parameterset 2" >> $zieldat
    # echo "[mrcges, Ifges] = inertial_parameters_convert_par1_par2(rSges, Icges, m);" >> $zieldat

    echo "%% Variable Initialization" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_If.m >> $zieldat
    
    printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi



  # Belegungsmatrix des Minimalparametervektors mit den Inertialparametern (Floating Base)
  quelldat1=$repo_pfad/codeexport/${robot_name}/PV2_MPV_transformation_linear_floatb_${basemeth}_matlab.m
  quelldat2=$repo_pfad/codeexport/${robot_name}/PV2_MPV_transformation_linear_dependant_floatb_${basemeth}_matlab.m
  quelldat3=$repo_pfad/codeexport/${robot_name}/PV2_permutation_linear_independant_floatb_${basemeth}_matlab.m
  quelldat4=$repo_pfad/codeexport/${robot_name}/PV2_permutation_linear_dependant_floatb_${basemeth}_matlab.m
  zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_PV2_MPV_transformations_floatb_${basemeth}.m
  if [ -f $quelldat1 ] && [ -f $quelldat2 ] && [ -f $quelldat3 ] && [ -f $quelldat4 ]; then
    cat ${tmp_pfad}_head/robot_matlabtmp_PV2_MPV_transformations_floatb_${basemeth}.head.m > $zieldat
    printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
    echo "%% Variable Initialization" >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
    echo "%% Symbolic Expressions" >> $zieldat
    printf "%%From ${quelldat1##*/}\n" >> $zieldat
    cat $quelldat1 >> $zieldat
    varname_tmp=`grep "=" $quelldat1 | tail -1 | sed 's/\(.*\)=.*/\1/'`
    echo "K = $varname_tmp;" >> $zieldat
    printf "%%From ${quelldat2##*/}\n" >> $zieldat
    cat $quelldat2 >> $zieldat
    varname_tmp=`grep "=" $quelldat2 | tail -1 | sed 's/\(.*\)=.*/\1/'`
    echo "K_d = $varname_tmp;" >> $zieldat
    printf "%%From ${quelldat3##*/}\n" >> $zieldat
    cat $quelldat3 >> $zieldat
    varname_tmp=`grep "=" $quelldat3 | tail -1 | sed 's/\(.*\)=.*/\1/'`
    echo "P_b = $varname_tmp;" >> $zieldat
    printf "%%From ${quelldat4##*/}\n" >> $zieldat
    cat $quelldat4 >> $zieldat
    varname_tmp=`grep "=" $quelldat4 | tail -1 | sed 's/\(.*\)=.*/\1/'`
    echo "P_d = $varname_tmp;" >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 0 0
  else
    echo "Code in ${quelldat1##*/} oder anderer nicht gefunden."
  fi




  # Generiere zwei verschiedene Regressorformen:
  # Minimalparameterregressor (rm=1) und Inertialparameterregressor (rm=2)
  for (( rm=1; rm<=2; rm++ ))
  do
    if [ $rm == 1 ]; then
      maple_string="regressor_minpar"
      matlab_string="regmin"
    else
      maple_string="regressor" 
      matlab_string="reg2" 
    fi

    # Kinetische Energie (Floating Base)
    quelldat=$repo_pfad/codeexport/${robot_name}/energy_kinetic_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_energykin_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_energykin_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
        if [ $basemeth == "twist" ]; then
          cat $tmp_pfad/robot_matlabtmp_assert_vB.m >> $zieldat
        else
          cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
          cat $tmp_pfad/robot_matlabtmp_assert_xDB.m >> $zieldat
        fi
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
        if [ $basemeth == "twist" ]; then
          cat $tmp_pfad/robot_matlabtmp_vB.m >> $zieldat
        else
          cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
          cat $tmp_pfad/robot_matlabtmp_xDB.m >> $zieldat
        fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Potentielle Energie (Floating Base)
    quelldat=$repo_pfad/codeexport/${robot_name}/energy_potential_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_energypot_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_energypot_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_rB.m >> $zieldat
        if [ $basemeth == "twist" ]; then
          :
        else
          cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
        fi
      cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_rB.m >> $zieldat
        if [ $basemeth == "twist" ]; then
          :
        else
          cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
        fi
      cat $tmp_pfad/robot_matlabtmp_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Gravitationsmoment (Basis)
    quelldat=$repo_pfad/codeexport/${robot_name}/base_gravload_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_gravloadB_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_gravloadB_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Gravitationsmoment (Gelenke)
    quelldat=$repo_pfad/codeexport/${robot_name}/joint_gravload_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_gravloadJ_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_gravloadJ_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Coriolisvektor (Floating Base, Basis)
    quelldat=$repo_pfad/codeexport/${robot_name}/coriolisvec_base_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_coriolisvecB_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_coriolisvecB_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_vB.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_xDB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        cat $tmp_pfad/robot_matlabtmp_vB.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_xDB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Coriolisvektor (Floating Base, Gelenke)
    quelldat=$repo_pfad/codeexport/${robot_name}/coriolisvec_joint_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_coriolisvecJ_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_coriolisvecJ_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_vB.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_xDB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        cat $tmp_pfad/robot_matlabtmp_vB.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_xDB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Coriolisvektor (Floating Base, Gesamt: Basis und Gelenke)
    quelldat=$repo_pfad/codeexport/${robot_name}/coriolisvec_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_coriolisvec_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_coriolisvec_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_vB.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_xDB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        cat $tmp_pfad/robot_matlabtmp_vB.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_xDB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi




    # Coriolismatrix (Floating Base: Gesamt)
    quelldat=$repo_pfad/codeexport/${robot_name}/coriolismat_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_coriolismat_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_coriolismat_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        cat $tmp_pfad/robot_matlabtmp_assert_vB.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_xDB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        cat $tmp_pfad/robot_matlabtmp_vB.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_xDB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
     
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi



    # Massenmatrix (Gesamt)
    quelldat=$repo_pfad/codeexport/${robot_name}/inertia_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_inertia_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_inertia_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Massenmatrix (Basis-Gelenke)
    quelldat=$repo_pfad/codeexport/${robot_name}/inertia_joint_base_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_inertiaJB_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_inertiaJB_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Massenmatrix (Basis)
    quelldat=$repo_pfad/codeexport/${robot_name}/inertia_base_base_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_inertiaB_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_inertiaB_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Massenmatrix-Zeitableitung (Floating Base: Gesamt)
    quelldat=$repo_pfad/codeexport/${robot_name}/inertiaD_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_inertiaD_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_inertiaD_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_xDB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_xDB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Inverse Dynamik der Gelenke (Floating Base)
    quelldat=$repo_pfad/codeexport/${robot_name}/invdyn_joint_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_invdynJ_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_invdynJ_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qDD.m >> $zieldat
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
     
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qDD.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        cat $tmp_pfad/robot_matlabtmp_vB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_aB.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_xDB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_xDDB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Inverse Dynamik der Basis (Floating Base)
    quelldat=$repo_pfad/codeexport/${robot_name}/invdyn_base_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}/${robot_name}_invdynB_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat ${tmp_pfad}_head/robot_matlabtmp_invdynB_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qD.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_qDD.m >> $zieldat
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
     
      printf "\n%%%% Variable Initialization" >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qD.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_qDD.m >> $zieldat
      if [ $basemeth == "twist" ]; then
        cat $tmp_pfad/robot_matlabtmp_vB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_aB.m >> $zieldat
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_xDB.m >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_xDDB.m >> $zieldat
      fi
      cat $tmp_pfad/robot_matlabtmp_g.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      
      printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi
  done
done
