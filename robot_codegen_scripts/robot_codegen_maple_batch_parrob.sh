#!/bin/bash -e
# Starte alle Maple-Skripte nacheinander in der richtigen Reihenfolge für parallelen Roboter
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Tim Job (Studienarbeit bei Moritz Schappler), 2018-12
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover

repo_pfad=$(pwd)/..
echo $repo_pfad

# Standard-Einstellungen
CG_PARALLEL=0
CG_MINIMAL=0
CG_FIXBONLY=0
CG_FLOATBONLY=0
CG_KINEMATICSONLY=0

# Argumente verarbeiten
# http://stackoverflow.com/questions/192249/how-do-i-parse-command-line-arguments-in-bash
while [[ $# > 0 ]]
do
key="$1"
case $key in
    -p|--parallel)
    CG_PARALLEL=1
    ;;
    --minimal)
    CG_MINIMAL=1
    ;;
    --fixb_only)
    CG_FIXBONLY=1
    ;;
    --floatb_only)
    CG_FLOATBONLY=1
    ;;
    --kinematics_only)
    CG_KINEMATICSONLY=1
    ;;
    *)
            # unknown option
    ;;
esac
shift # past argument or value
done

# Namen des Roboters herausfinden (damit roboterspezifische Zwangsbedingungen berechnet werden können)
source robot_codegen_tmpvar_bash_par.sh
source $repo_pfad/robot_codegen_definitions/robot_env_par.sh

# Aktuellen Modus speichern (für Arbeitsblätter, die im IC- oder Normal-Modus benutzt werden)
echo "tbmode := \"parrob\":" > $repo_pfad/workdir/tbmode

# Liste mit Maple-Skripten in der richtigen Reihenfolge für parallelen Roboter
dateiliste_kin="
  robot_para_definitions.mpl
  robot_para_rotmat_kinematics.mpl
  "
for mpldat in $dateiliste_kin
do
  mpldat_full=$repo_pfad/workdir/$mpldat
  $repo_pfad/scripts/run_maple_script.sh $mpldat_full
done

if [ "$CG_KINEMATICSONLY" == "0" ]; then
  if [ "$CG_MINIMAL" == "0" ]; then
    dateiliste_dyn="
      robot_para_plattform_rotmat_dynamics_par1.mpl
      robot_para_rotmat_projection_dynamics_par1.mpl"
    for mpldat in $dateiliste_dyn
    do
      mpldat_full=$repo_pfad/workdir/$mpldat
      $repo_pfad/scripts/run_maple_script.sh $mpldat_full
    done
    
    dateiliste_dynexport="
      robot_para_rotmat_projection_dynamics_export_par1_invdyn.mpl
      robot_para_rotmat_projection_dynamics_export_par1_corvec.mpl
      robot_para_rotmat_projection_dynamics_export_par1_grav.mpl
      robot_para_rotmat_projection_dynamics_export_par1_inertia.mpl"
    for mpldat in $dateiliste_dynexport
    do
      mpldat_full=$repo_pfad/workdir/$mpldat
      if [ "$CG_PARALLEL" == "0" ]; then
        $repo_pfad/scripts/run_maple_script.sh $mpldat_full
      else
        $repo_pfad/scripts/run_maple_script.sh $mpldat_full &
      fi
    done
    wait
  fi;
  
  dateiliste_dyn="
    robot_para_plattform_rotmat_dynamics_par2.mpl
    robot_para_rotmat_projection_dynamics_par2.mpl"
  for mpldat in $dateiliste_dyn
  do
    mpldat_full=$repo_pfad/workdir/$mpldat
    $repo_pfad/scripts/run_maple_script.sh $mpldat_full
  done
  dateiliste_dynexport="
    robot_para_rotmat_projection_dynamics_export_par2_invdyn.mpl
    robot_para_rotmat_projection_dynamics_export_par2_corvec.mpl
    robot_para_rotmat_projection_dynamics_export_par2_grav.mpl
    robot_para_rotmat_projection_dynamics_export_par2_inertia.mpl"
  for mpldat in $dateiliste_dynexport
  do
    mpldat_full=$repo_pfad/workdir/$mpldat
    if [ "$CG_PARALLEL" == "0" ]; then
      $repo_pfad/scripts/run_maple_script.sh $mpldat_full
    else
      $repo_pfad/scripts/run_maple_script.sh $mpldat_full &
    fi
  done
  # wait # Auf den Code-Export braucht nicht gewartet werden
  if [ "$CG_MINIMAL" == "0" ]; then
    dateiliste_reg="
      robot_para_plattform_rotmat_dynamics_regressor.mpl
      robot_para_rotmat_projection_dynamics_regressor_pv2.mpl
      robot_para_rotmat_projection_dynamics_regressor_minpar.mpl"
    for mpldat in $dateiliste_reg
    do
      mpldat_full=$repo_pfad/workdir/$mpldat
      $repo_pfad/scripts/run_maple_script.sh $mpldat_full
    done
    dateiliste_regexport="
      robot_tree_base_parameter_transformations.mpl
      robot_para_rotmat_projection_dynamics_regressor_export_pv2_invdyn.mpl
      robot_para_rotmat_projection_dynamics_regressor_export_pv2_corvec.mpl
      robot_para_rotmat_projection_dynamics_regressor_export_pv2_grav.mpl
      robot_para_rotmat_projection_dynamics_regressor_export_pv2_inertia.mpl
      robot_para_rotmat_projection_dynamics_regressor_export_minpar_invdyn.mpl
      robot_para_rotmat_projection_dynamics_regressor_export_minpar_corvec.mpl
      robot_para_rotmat_projection_dynamics_regressor_export_minpar_grav.mpl
      robot_para_rotmat_projection_dynamics_regressor_export_minpar_inertia.mpl"
    for mpldat in $dateiliste_regexport
    do
      mpldat_full=$repo_pfad/workdir/$mpldat
      if [ "$CG_PARALLEL" == "0" ]; then
        $repo_pfad/scripts/run_maple_script.sh $mpldat_full
      else
        $repo_pfad/scripts/run_maple_script.sh $mpldat_full &
      fi
    done
    wait
  fi;
  wait # Warten auf eventuell nicht beendeten Code-Export der Dynamik
fi;

