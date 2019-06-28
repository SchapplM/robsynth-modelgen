#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Dynamik IC (Explizit/Regressorform/Lagrange/Newton), fixed/floating-base"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
template_pfad=$repo_pfad/robot_codegen_scripts/templates_sym
# Initialisiere Variablen
source robot_codegen_tmpvar_bash_ic.sh
source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh

# Schleife für beide Basis-Darstellungen durchgehen.
# Die Methode "twist" muss am Ende kommen, damit Basis-unabhängige Funktionen mit den Ergebnissen dieser Methode
# generiert werden ("twist"-Code ist kürzer; Zuerst erstellte Datei mit "eulxyz" wird dann überschrieben).
basemethodenames=( twist eulxyz )

# Erstelle Matlab-Funktionen der explizit ausgerechneten Dynamik (nicht in Regressorform)

for (( dynpar=1; dynpar<=2; dynpar++ ))
do
  for basemeth in "${basemethodenames[@]}"
  do
    
    # Inverse Dynamik (Floating Base)
    source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_floatb_${basemeth}_par${dynpar}_ic_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdynJ_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_invdynJ_floatb_ic_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      sed -i "s/%RN%/$robot_name/g" $zieldat
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
      sed -i "s/%NAJ%/$robot_NAJ/g" $zieldat
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi
    
    # Inverse Dynamik (Fixed Base)
    source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_fixb_par${dynpar}_ic_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdynJ_fixb_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_invdynJ_fixb_ic_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      sed -i "s/%RN%/$robot_name/g" $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
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
      
      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJDD.m >> ${quelldat}.subsvar
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
      sed -i "s/%NAJ%/$robot_NAJ/g" $zieldat
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi  
    
    # Massenmatrix (Gelenke)
    source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/inertia_joint_joint_floatb_${basemeth}_par${dynpar}_ic_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_inertiaJ_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_inertiaJ_ic_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      sed -i "s/%RN%/$robot_name/g" $zieldat
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
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
      
      printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
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
      sed -i "s/%NAJ%/$robot_NAJ/g" $zieldat
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi
    
    # Gravitationsmoment (Gelenke)
    source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/gravload_joint_floatb_${basemeth}_par${dynpar}_ic_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_gravloadJ_floatb_${basemeth}_slag_vp${dynpar}.m
    if [ -f $quelldat ]; then
      cat $head_pfad/robot_matlabtmp_gravloadJ_floatb_${basemeth}_ic_par${dynpar}.head.m > $zieldat
      printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      sed -i "s/%RN%/$robot_name/g" $zieldat
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
      sed -i "s/%NAJ%/$robot_NAJ/g" $zieldat
    else
      echo "Code in ${quelldat##*/} nicht gefunden."
    fi
    
  done # floatb_twist/floatb_eulxyz
done # par1/par2

# for basemeth in "${basemethodenames[@]}"
# do
  # # Minimalparameterregressor (rm=1) und Inertialparameterregressor (rm=2)
  # for (( rm=2; rm<=2; rm++ ))
  # do
    # if [ $rm == 1 ]; then
      # maple_string_reg="regressor_minpar"
      # matlab_string_reg="regmin"
    # else
      # maple_string_reg="regressor"
      # matlab_string_reg="reg2"
    # fi
    
    # # Inverse Dynamik in Regressorform (Floating Base)
    # source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh
    # quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_floatb_${basemeth}_${maple_string_reg}_ic_matlab.m
    # zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdynJ_floatb_${basemeth}_${matlab_string_reg}_slag_vp.m
    # if [ -f $quelldat ]; then
      # cat $head_pfad/robot_matlabtmp_invdynJ_floatb_${basemeth}_ic_${matlab_string}.head.m > $zieldat
      # printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      # sed -i "s/%RN%/$robot_name/g" $zieldat
      # source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      # source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      # cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      # cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
      # cat $tmp_pfad/robot_matlabtmp_assert_qJDD.m >> $zieldat
      # if [ $basemeth == "twist" ]; then
        # cat $tmp_pfad/robot_matlabtmp_assert_vB.m >> $zieldat
        # cat $tmp_pfad/robot_matlabtmp_assert_aB.m >> $zieldat
      # else
        # cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
        # cat $tmp_pfad/robot_matlabtmp_assert_xDB.m >> $zieldat
        # cat $tmp_pfad/robot_matlabtmp_assert_xDDB.m >> $zieldat
      # fi
      # cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      # cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
     
      # printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      # cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      # cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
      # cat $tmp_pfad/robot_matlabtmp_qJDD.m >> ${quelldat}.subsvar
      # if [ $basemeth == "twist" ]; then
        # cat $tmp_pfad/robot_matlabtmp_vB.m >> ${quelldat}.subsvar
        # cat $tmp_pfad/robot_matlabtmp_aB.m >> ${quelldat}.subsvar
      # else
        # cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
        # cat $tmp_pfad/robot_matlabtmp_xDB.m >> ${quelldat}.subsvar
        # cat $tmp_pfad/robot_matlabtmp_xDDB.m >> ${quelldat}.subsvar
      # fi
      # cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      # cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      
      # printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      # sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      # cat $quelldat >> $zieldat
      # source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
      # sed -i "s/%NAJ%/$robot_NAJ/g" $zieldat
    # else
      # echo "Code in ${quelldat##*/} nicht gefunden."
    # fi
    
    # # Inverse Dynamik in Regressorform (Fixed Base)
    # source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh
    # quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_fixb_${maple_string_reg}_ic_matlab.m
    # zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdynJ_fixb_${matlab_string_reg}_slag_vp.m
    # if [ -f $quelldat ]; then
      # cat $head_pfad/robot_matlabtmp_invdynJ_fixb_ic_${matlab_string_reg}.head.m > $zieldat
      # printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
      # sed -i "s/%RN%/$robot_name/g" $zieldat
      # source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
      # source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
      # cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
      # cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
      # cat $tmp_pfad/robot_matlabtmp_assert_qJDD.m >> $zieldat
      # cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
      # cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      # printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
      # cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
      # cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
      # cat $tmp_pfad/robot_matlabtmp_qJDD.m >> ${quelldat}.subsvar
      # cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
      # cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
      # printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
      # sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
      # cat $quelldat >> $zieldat
      # source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
      # sed -i "s/%NAJ%/$robot_NAJ/g" $zieldat
    # else
      # echo "Code in ${quelldat##*/} nicht gefunden."
    # fi
    
  # done # Minimalparameterregressor/Inertialparameterregressor
# done # floatb_twist/floatb_eulxyz

# Newton-Euler
for (( dynpar=1; dynpar<=2; dynpar++ ))
do
  # Inverse Dynamik (Fixed Base)
  source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh
  quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_fixb_snew_par${dynpar}_ic_matlab.m
  zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdynJ_fixb_snew_vp${dynpar}.m
  if [ -f $quelldat ]; then
    cat $head_pfad/robot_matlabtmp_invdynJ_fixb_snew_ic_par${dynpar}.head.m > $zieldat
    printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
    sed -i "s/%RN%/$robot_name/g" $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
    source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
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
    
    printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
    cat $tmp_pfad/robot_matlabtmp_qJDD.m >> ${quelldat}.subsvar
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
    sed -i "s/%NAJ%/$robot_NAJ/g" $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi  
  
  # Gravitationsmoment (Gelenke)
  source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh
  quelldat=$repo_pfad/codeexport/${robot_name}/tmp/gravload_joint_floatb_twist_snew_par${dynpar}_ic_matlab.m
  zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_gravloadJ_floatb_twist_snew_vp${dynpar}.m
  if [ -f $quelldat ]; then
    cat $head_pfad/robot_matlabtmp_gravloadJ_floatb_twist_snew_ic_par${dynpar}.head.m > $zieldat
    printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
    sed -i "s/%RN%/$robot_name/g" $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
    source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
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
    sed -i "s/%NAJ%/$robot_NAJ/g" $zieldat
  else
    echo "Code in ${quelldat##*/} nicht gefunden."
  fi
done
