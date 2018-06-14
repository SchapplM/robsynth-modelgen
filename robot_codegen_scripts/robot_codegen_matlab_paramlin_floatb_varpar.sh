#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur parameterlinearen Form der Floating-Base-Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2017-02
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Dynamik-Regressor (Floating Base)"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

basemethodenames=( eulangrpy )

# Erstelle Matlab-Funktionen der Regressorform für Floating Base Modell
for basemeth in "${basemethodenames[@]}"
do
  # Minimalparametervektor (Floating Base)
  quelldat=$repo_pfad/codeexport/${robot_name}/tmp/minimal_parameter_vector_floatb_${basemeth}_matlab.m
  zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_convert_par2_MPV_floatb_${basemeth}.m
  if [ -f $quelldat ]; then
    cat $head_pfad/robot_matlabtmp_convert_par2_MPV_floatb_${basemeth}.head.m > $zieldat
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
    sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
    cat $quelldat >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi



  # Belegungsmatrix des Minimalparametervektors mit den Inertialparametern (Floating Base)
  quelldat1=$repo_pfad/codeexport/${robot_name}/tmp/PV2_MPV_transformation_linear_floatb_${basemeth}_matlab.m
  quelldat2=$repo_pfad/codeexport/${robot_name}/tmp/PV2_MPV_transformation_linear_dependant_floatb_${basemeth}_matlab.m
  quelldat3=$repo_pfad/codeexport/${robot_name}/tmp/PV2_permutation_linear_independant_floatb_${basemeth}_matlab.m
  quelldat4=$repo_pfad/codeexport/${robot_name}/tmp/PV2_permutation_linear_dependant_floatb_${basemeth}_matlab.m
  zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_PV2_MPV_transformations_floatb_${basemeth}.m
  if [ -f $quelldat1 ] && [ -f $quelldat2 ] && [ -f $quelldat3 ] && [ -f $quelldat4 ]; then
    cat $head_pfad/robot_matlabtmp_PV2_MPV_transformations_floatb_${basemeth}.head.m > $zieldat
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
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/energy_kinetic_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_energykin_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_energykin_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
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
      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
        if [ $basemeth == "twist" ]; then
          cat $tmp_pfad/robot_matlabtmp_vB.m >> ${quelldat}.subsvar
        else
          cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
          cat $tmp_pfad/robot_matlabtmp_xDB.m >> ${quelldat}.subsvar
        fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Potentielle Energie (Floating Base)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/energy_potential_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_energypot_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_energypot_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
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
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Gravitationsmoment (Basis)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/base_gravload_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_gravloadB_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_gravloadB_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
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
      
      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
      fi
      cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Gravitationsmoment (Gelenke)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/joint_gravload_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_gravloadJ_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_gravloadJ_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
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
      
      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
      fi
      cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Coriolisvektor (Floating Base, Basis)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/coriolisvec_base_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_coriolisvecB_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_coriolisvecB_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
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
      
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Coriolisvektor (Floating Base, Gelenke)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/coriolisvec_joint_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_coriolisvecJ_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_coriolisvecJ_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
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
      
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Coriolisvektor (Floating Base, Gesamt: Basis und Gelenke)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/coriolisvec_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_coriolisvec_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_coriolisvec_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
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
      
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi




    # Coriolismatrix (Floating Base: Gesamt)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/coriolismat_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_coriolismat_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_coriolismat_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
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
     
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi



    # Massenmatrix (Gesamt)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/inertia_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_inertia_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_inertia_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
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
      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Massenmatrix (Basis-Gelenke)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/inertia_joint_base_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_inertiaJB_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_inertiaJB_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
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
      
      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi


    # Massenmatrix (Basis)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/inertia_base_base_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_inertiaB_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_inertiaB_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
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
      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      if [ $basemeth == "twist" ]; then
        :
      else
        cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
      fi
      cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Massenmatrix-Zeitableitung (Floating Base: Gesamt)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/inertiaD_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_inertiaD_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_inertiaD_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
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
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Inverse Dynamik der Gelenke (Floating Base)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_joint_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdynJ_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_invdynJ_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
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
      
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi

    # Inverse Dynamik der Basis (Floating Base)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_base_floatb_${basemeth}_${maple_string}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdynB_floatb_${basemeth}_${matlab_string}_slag_vp.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_invdynB_floatb_${basemeth}_${matlab_string}.head.m > $zieldat
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
      
      printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      cat $quelldat >> $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi
  done
done
