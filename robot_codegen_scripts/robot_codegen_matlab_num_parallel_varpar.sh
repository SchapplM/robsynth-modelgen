#!/bin/bash -e
# Passe die Vorlagen für numerische Funktionen an den parallelen Roboter an.
# Diese Funktionen sind durch assert-Befehle und hart kodierte
# Zahlen an den Roboter angepasst und dadurch kompilierbar.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt
#
# Siehe auch: robot_codegen_matlab_num_varpar.sh (davon abgeleitet)

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-01
# (C) Institut für Mechatronische Systeme, Universität Hannover

echo "Generiere diverse PKM-Funktionen (numerisch)"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
template_pfad=$repo_pfad/robot_codegen_scripts/templates_par

# Initialisiere Variablen
source robot_codegen_tmpvar_bash_par.sh
source $repo_pfad/robot_codegen_definitions/robot_env_par.sh

fcn_pfad=$repo_pfad/codeexport/$robot_name/matlabfcn
test_pfad=$repo_pfad/codeexport/$robot_name/testfcn

# Vorlagen aus Robotik-Repo kopieren (falls verfügbar)
# Benutze eine separate Datei zum Zeigen auf den Pfad, falls unter Windows.
linkfiles="$repo_pfad/robotics_repo_path_linux
  $repo_pfad/robotics_repo_path"
for linkfile in $linkfiles; do
if [ -f $linkfile ]; then
  robrepopath=`sed -n -e 's/^robotics_repo_path := "\(.*\)":/\1/p' $linkfile`
  if [ "$robrepopath" == "" ]; then
    echo "Ausdruck "robotics_repo_path= \"/pfad/zum/repo\":" in $linkfile nicht gefunden"
    continue
  fi
  if [ -d $robrepopath ]; then
    for f in `find "$robrepopath/kinematics" -name 'pkm_*.template'`; do
      filename="${f##*/}"
      filename=`echo "$filename" | sed s/pkm/robot/`
      if [[ -L $template_pfad/$filename ]]; then
        continue # Symbolischen Link erhalten (sonst Kopierfehler)
      fi
      cp $f $template_pfad/$filename
      # Nachbearbeitung der Vorlagen-Funktionen. Die Ersetzungsausdrücke
      # unterscheiden sich zwischen diesem Repo und dem Robotik-Repo.
      sed -i "s/%NLEG%/%N_LEGS%/g" $template_pfad/$filename
      sed -i "s/%NJ%/%NJ_PKM%/g" $template_pfad/$filename
      sed -i "s/%NL%/%NL_PKM%/g" $template_pfad/$filename
      sed -i "s/%PN%/%RN%/g" $template_pfad/$filename
      sed -i "s/%SN%/%LEGNAME%/g" $template_pfad/$filename
    done
    break # Nur eine Datei nutzen. Die erste gültige reicht.
  else
    echo "Datei robotics_repo_path aus $linkfile enthält kein gültiges Verzeichnis."
  fi;
else
  echo "Datei robotics_repo_path aus $linkfile existiert nicht"
fi
done


# Alle Vorlagen an den Roboter anpassen und kopieren
for f in $(find $template_pfad -name "*.template")
do
  tmpdat_full=$f
  filename="${tmpdat_full##*/}"                      # Strip longest match of */ from start
  dir="${tmpdat_full:0:${#tmpdat_full} - ${#filename} - 1}" # Substring from 0 thru pos of filename

  # Neuen Dateinamen generieren
  tmp="${filename/robot/$robot_name}"
  filename_new="${tmp/.template/}"
  
  # Datei kopieren
  cp $dir/$filename $fcn_pfad/$filename_new

  # Platzhalter in Datei ersetzen
  source robot_codegen_matlabfcn_postprocess_par.sh $fcn_pfad/$filename_new 0
done


# Erzeuge Dynamik-Parameter-Funktion. Damit werden benutzerspezifische Dynamikparameter
# für die Testskripte erzeugt (z.B. wenn einzelne Parameter Null gesetzt werden)
zieldat=$fcn_pfad/${robot_name}_dynamics_parameters_modification.m # Die Dateikopf-Vorlage wird automatisch hier angelegt
zieldat2=$test_pfad/${robot_name}_dynamics_parameters_modification.m # Die Datei soll am Ende hier liegen
printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
cat $tmp_pfad/robot_matlabtmp_assert_m_parallel.m >> $zieldat
cat $tmp_pfad/robot_matlabtmp_assert_rcom_parallel.m >> $zieldat
cat $tmp_pfad/robot_matlabtmp_assert_Ic_parallel.m >> $zieldat

printf "\n%%%% Variable Initialization" >> $zieldat
printf "\n%% Complete set of dynamics parameters that can be reduced by the user." >> $zieldat
cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
cat $tmp_pfad/robot_matlabtmp_par_m_parallel.m >> $zieldat
cat $tmp_pfad/robot_matlabtmp_par_rcom_parallel.m >> $zieldat
cat $tmp_pfad/robot_matlabtmp_par_Ic_parallel.m >> $zieldat

printf "\n%%%% Parameter Postprocessing / Set Output" >> $zieldat
printf "\n%% Create the reduced set of dynamics parameters that is also used for generation of the dynamics equations." >> $zieldat

printf "\n%% Aus parameters_dyn_mges_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name}/tmp/parameters_dyn_mges_pkm_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "m = $varname_tmp;" >> $zieldat

printf "\n%% Aus parameters_dyn_rSges_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name}/tmp/parameters_dyn_rSges_pkm_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "rSges = $varname_tmp;" >> $zieldat

printf "\n%% Aus parameters_dyn_Icges_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name}/tmp/parameters_dyn_Icges_pkm_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "Icges = $varname_tmp;" >> $zieldat

# Platzhalter ersetzen
source robot_codegen_matlabfcn_postprocess_par.sh $zieldat 0
# Funktion in Ordner für Testskripte verschieben (wird nur dafür benötigt)
mv $zieldat $zieldat2
