#!/bin/bash -e
# Starte alle Maple-Skripte in der richtigen Reihenfolge
# Führe so viele Berechnungen wie möglich parallel aus
#
# Argumente:
# --minimal
#   Berechne nur minimale Anzahl an Funktionen
# --fixb_only
#   Nur Berechnung der Fixed-Base Funktionen.
# --floatb_only
#   Nur Berechnung der Fixed-Base Funktionen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/..
workdir=$repo_pfad/workdir


# Standard-Einstellungen
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

if [ "$CG_FIXBONLY" == "1" ] && [ "$CG_FLOATBONLY" == "1" ]; then
  echo "Nicht beide Optionen gleichzeitig möglich: fixb_only, floatb_only"
  exit 1
fi;

# Namen des Roboters herausfinden (damit roboterspezifische Zwangsbedingungen berechnet werden können)
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Aktuellen Modus speichern (für Arbeitsblätter, die im IC- oder Normal-Modus benutzt werden)
echo "tbmode := \"serial\":" > $repo_pfad/workdir/tbmode

if [ "$CG_FIXBONLY" == "1" ]; then
  # Berechne alles nur für fixed-base Modellierung (dafür reicht die Methode "twist")
  basemethodenames=( twist )
elif [ "$CG_FLOATBONLY" == "1" ]; then
  # Berechne nur floating base Modellierung
  basemethodenames=( eulxyz )
else
  # Berechne beides
  basemethodenames=( twist eulxyz )
fi;

for basemeth in "${basemethodenames[@]}"
do
  dateiliste_kin="
      robot_tree_floatb_${basemeth}_definitions.mpl
  "
   if [ -f  $repo_pfad/workdir/${robot_name}_kinematic_constraints.mpl ]; then
    dateiliste_kin="$dateiliste_kin
      ${robot_name}_kinematic_constraints.mpl
      robot_kinematic_constraints_calculations.mpl
    "
  fi;
  dateiliste_kin="$dateiliste_kin
      robot_tree_kinematic_parameter_list.mpl
      robot_tree_floatb_rotmat_mdh_kinematics.mpl
      robot_tree_floatb_rotmat_kinematics_com_worldframe_par1.mpl
  "
  dateiliste_mdhvel="
      robot_tree_velocity_mdh_angles.mpl
  "

  dateiliste_vel="
      robot_tree_floatb_rotmat_velocity_worldframe_par1.mpl
      robot_tree_floatb_rotmat_velocity_linkframe.mpl
      robot_tree_acceleration_mdh_angles.mpl
  "
  
  if [ "$CG_KINEMATICSONLY" == "0" ]; then
    if [ "$CG_MINIMAL" == "0" ]; then
      dateiliste_en="
          robot_tree_floatb_rotmat_energy_worldframe_par1.mpl
          robot_tree_floatb_rotmat_energy_worldframe_par2.mpl
          robot_tree_floatb_rotmat_energy_linkframe_par2.mpl
      "
    else
      dateiliste_en="
          robot_tree_floatb_rotmat_energy_worldframe_par2.mpl
          robot_tree_floatb_rotmat_energy_linkframe_par2.mpl
      "
    fi;
    dateiliste_dyndep="
        robot_tree_floatb_rotmat_lagrange_worldframe_par2.mpl
    "
    if [ "$CG_MINIMAL" == "0" ]; then
      dateiliste_dyndep="
          $dateiliste_dyndep
          robot_tree_floatb_rotmat_lagrange_worldframe_par1.mpl
      "
    fi;
    dateiliste_dyn1="
        robot_tree_floatb_rotmat_dynamics_worldframe_par2_inertia.mpl
    "
    dateiliste_dyn2="
        robot_tree_floatb_rotmat_dynamics_worldframe_par2_corvec.mpl
        robot_tree_floatb_rotmat_dynamics_worldframe_par2_grav.mpl
    "
    if [ "$robot_kinconstr_exist" == "0" ]; then
          dateiliste_dyn2="$dateiliste_dyn2
          robot_tree_fixb_dynamics_NewtonEuler_linkframe_par12.mpl
      "
        dateiliste_acc="
          robot_tree_floatb_rotmat_acceleration_linkframe.mpl
      "
    fi;
    if [ "$CG_MINIMAL" == "1" ]; then
      # Im Minimal-Modus wird nichts berechnet, das von der Massenmatrix abhängt.
      # Es kann alles parallel gerechnet werden.
      dateiliste_dyn1="
          $dateiliste_dyn1
          $dateiliste_dyn2
      "
      dateiliste_dyn2=""
    fi
    if [ "$CG_MINIMAL" == "0" ]; then
      # Nur Massenmatrix in erstem Skript berechnen
      dateiliste_dyn1="
          $dateiliste_dyn1
          robot_tree_floatb_rotmat_dynamics_worldframe_par1_inertia.mpl
      "
      # Danach alles andere berechnen (cormat und inertiaD von inertia abhängig)
      dateiliste_dyn2="
          $dateiliste_dyn2
          robot_tree_floatb_rotmat_dynamics_worldframe_par1_corvec.mpl
          robot_tree_floatb_rotmat_dynamics_worldframe_par1_cormat.mpl
          robot_tree_floatb_rotmat_dynamics_worldframe_par1_grav.mpl
          robot_tree_floatb_rotmat_dynamics_worldframe_par1_inertiaD.mpl
          robot_tree_floatb_rotmat_dynamics_worldframe_par1_invdyn.mpl
          robot_tree_floatb_rotmat_dynamics_worldframe_par2_cormat.mpl
          robot_tree_floatb_rotmat_dynamics_worldframe_par2_invdyn.mpl
          robot_tree_floatb_rotmat_dynamics_worldframe_par2_inertiaD.mpl
      "
    fi;
  
    if [ "$CG_MINIMAL" == "0" ]; then
      # Nur Linearisierung und Parameterminimierung in erstem Skript
      if [ ${basemeth} == "twist" ]; then
          dateiliste_plin1="
              robot_chain_fixb_rotmat_energy_regressor.mpl
              robot_chain_fixb_energy_regressor_linearsolve.mpl"
      else
          dateiliste_plin1="robot_chain_floatb_rotmat_energy_regressor.mpl"
      fi;
      # darauf aufbauende Berechnung der Dynamik in zweitem Durchlauf
      dateiliste_plin2="
        robot_tree_base_parameter_transformations.mpl
        robot_chain_floatb_rotmat_dynamics_regressor_minpar_corvec.mpl
        robot_chain_floatb_rotmat_dynamics_regressor_minpar_cormat.mpl
        robot_chain_floatb_rotmat_dynamics_regressor_minpar_grav.mpl
        robot_chain_floatb_rotmat_dynamics_regressor_minpar_inertia.mpl
        robot_chain_floatb_rotmat_dynamics_regressor_minpar_inertiaD.mpl
        robot_chain_floatb_rotmat_dynamics_regressor_minpar_invdyn.mpl
        robot_chain_floatb_rotmat_dynamics_regressor_pv2_corvec.mpl
        robot_chain_floatb_rotmat_dynamics_regressor_pv2_cormat.mpl
        robot_chain_floatb_rotmat_dynamics_regressor_pv2_grav.mpl
        robot_chain_floatb_rotmat_dynamics_regressor_pv2_inertia.mpl
        robot_chain_floatb_rotmat_dynamics_regressor_pv2_inertiaD.mpl
        robot_chain_floatb_rotmat_dynamics_regressor_pv2_invdyn.mpl
      "
      if [ "$robot_kinconstr_exist" == "0" ]; then
            dateiliste_plin2="$dateiliste_plin2
              robot_chain_fixb_rotmat_NewtonEuler_regressor.mpl
        "
      fi;
    fi;
  fi;
  
  # Zusätzliche Maple-Skripte speziell für dieses System (benutzerdefiniert)
  # Für jede Basis-Methode anhängen.
  addlistfile=$repo_pfad/robot_codegen_additional/scripts/${robot_name}_maple_additional_worksheet_list_${basemeth}
  if [ -f $addlistfile ]; then
    dateiliste_add="$dateiliste_kindyn `cat $addlistfile`"
  else
    dateiliste_add=""
  fi;


  # Alle Arbeitsblätter parallel ausführen, wo dies möglich ist
  for wskin in ${dateiliste_kin[@]}
  do
    mpldat_full=$workdir/$wskin
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    echo "Starte Maple-Skript $filename"
    $repo_pfad/scripts/run_maple_script.sh $dir/$filename
  done
  echo "FERTIG mit Kinematik für ${basemeth}"
  for wsvelm in ${dateiliste_mdhvel[@]}
  do
    mpldat_full=$workdir/$wsvelm
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    echo "Starte Maple-Skript $filename"
    $repo_pfad/scripts/run_maple_script.sh $dir/$filename
  done
  for wsvel in ${dateiliste_vel[@]}
  do
    mpldat_full=$workdir/$wsvel
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    echo "Starte Maple-Skript $filename"
    $repo_pfad/scripts/run_maple_script.sh $dir/$filename &
  done
  wait
  echo "FERTIG mit Geschwindigkeit für ${basemeth}"
  if [ "$robot_kinconstr_exist" == "0" ]; then
    for wsacc in ${dateiliste_acc[@]}
    do
      mpldat_full=$workdir/$wsacc
      filename="${mpldat_full##*/}"
      dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
      echo "Starte Maple-Skript $filename"
      $repo_pfad/scripts/run_maple_script.sh $dir/$filename &
    done
    wait
    echo "FERTIG mit Beschleunigung für ${basemeth}"
  fi;
  for wsen in ${dateiliste_en[@]}
  do
    mpldat_full=$workdir/$wsen
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    echo "Starte Maple-Skript $filename"
    $repo_pfad/scripts/run_maple_script.sh $dir/$filename &
  done
  wait
  echo "FERTIG mit Energie für ${basemeth}"
  
  for wslag in ${dateiliste_dyndep[@]}
  do
    mpldat_full=$workdir/$wslag
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    echo "Starte Maple-Skript $filename"
    $repo_pfad/scripts/run_maple_script.sh $dir/$filename &
  done
  wait
  echo "FERTIG mit Lagrange für ${basemeth}"

  for wsdyn in ${dateiliste_dyn1[@]}
  do
    mpldat_full=$workdir/$wsdyn
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    echo "Starte Maple-Skript $filename"
    $repo_pfad/scripts/run_maple_script.sh $dir/$filename &
  done
  wait
  echo "FERTIG mit Dynamik Teil 1 für ${basemeth}"
  for wsdyn in ${dateiliste_dyn2[@]}
  do
    mpldat_full=$workdir/$wsdyn
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    echo "Starte Maple-Skript $filename"
    $repo_pfad/scripts/run_maple_script.sh $dir/$filename &
  done
  wait
  echo "FERTIG mit Dynamik Teil 2 für ${basemeth}"
  for wsplin in ${dateiliste_plin1[@]}
  do
    mpldat_full=$workdir/$wsplin
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    # nicht parallel ausführen
    echo "Starte Maple-Skript $filename"
    $repo_pfad/scripts/run_maple_script.sh $dir/$filename
  done
  for wsplin in ${dateiliste_plin2[@]}
  do
    mpldat_full=$workdir/$wsplin
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    # parallel ausführen
    echo "Starte Maple-Skript $filename"
    $repo_pfad/scripts/run_maple_script.sh $dir/$filename &
  done
  wait
  echo "FERTIG mit Regressorform für ${basemeth}"
done

# Definitionen des Fixed-Base-Modell wieder laden (für Jacobi-Matrizen und zusätzliche Dateien)
echo "Starte Maple-Skript robot_tree_floatb_twist_definitions.mpl"
$repo_pfad/scripts/run_maple_script.sh $repo_pfad/workdir/robot_tree_floatb_twist_definitions.mpl

# Kinematische Zwangsbedingungen in impliziter Form
# Werden nach der Kinematik gerechnet. Können also auch hier am Ende kommen
# Die Ergebnisse werden in der Dynamik nicht weiter benutzt (im Gegensatz zu explizit definierten Zwangsbedingungen, die direkt zur Ersetzung dienen).
if [ -f  $repo_pfad/robot_codegen_constraints/${robot_name}_kinematic_constraints_implicit.mpl ]; then
  dateiliste_impconstr="
    ${robot_name}_kinematic_constraints_implicit.mpl
    robot_kinematic_constraints_calculations_implicit.mpl
  "
  for wsic in ${dateiliste_impconstr[@]}
  do
    mpldat_full=$workdir/$wsic
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    echo "Starte Maple-Skript $filename"
    $repo_pfad/scripts/run_maple_script.sh $dir/$filename &
  done
fi;


# Jacobi-Matrizen
dateiliste_jac=""
for (( ib=0; ib<$robot_NL; ib++ ))
do
  dateiliste_jac="$dateiliste_jac
        /robot_tree_rotmat_jacobian_baseframe_body${ib}.mpl
  "
done
for wsjac in ${dateiliste_jac[@]}
do
  mpldat_full=$workdir/$wsjac
  filename="${mpldat_full##*/}"
  dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
  echo "Starte Maple-Skript $filename"
  $repo_pfad/scripts/run_maple_script.sh $dir/$filename &
done
wait
echo "FERTIG mit Jacobi-Matrizen"

if [ -f $addlistfile ]; then
  for wsadd in ${dateiliste_add[@]}
  do
    mpldat_full=$workdir/$wsadd
    filename="${mpldat_full##*/}"
    dir="${mpldat_full:0:${#mpldat_full} - ${#filename} - 1}"
    echo "Starte Maple-Skript $filename"
    $repo_pfad/scripts/run_maple_script.sh $dir/$filename &
  done
  echo "Zusätzlichen Dateien gestartet"
fi;

wait
echo "Alle Matlab-Funktionen exportiert"
