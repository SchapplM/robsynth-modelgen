#!/bin/bash -e
# Passe die Vorlagen für numerische Funktionen an den Roboter an.
# Diese Funktionen sind durch assert-Befehle und hart kodierte
# Zahlen an den Roboter angepasst und dadurch kompilierbar.
# Die allgemeinen Dynamikfunktionen aus der Matlab-Toolbox
# sind nicht kompilierbar und damit etwas langsamer.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt
#
# Eingabeargumente:
# Argument 1: "1", wenn nur Kinematik generiert werden soll

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-06
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Dynamikfunktion (numerisch)"

repo_pfad=$(pwd)/..
template_pfad=$repo_pfad/robot_codegen_scripts/templates_num

kinematicsOnly=$1

# Standardwerte für Eingabe
if [ "$kinematicsOnly" == "" ]; then
  kinematicsOnly=0
fi

# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

fcn_pfad=$repo_pfad/codeexport/$robot_name/matlabfcn
test_pfad=$repo_pfad/codeexport/$robot_name/testfcn

# Erstelle Liste mit nicht zu generierenden Dateien
blacklist=""
# Für Systeme mit kinematischen Zwangsbedingungen funktionieren einige Ansätze der nicht
# (z.B. Inverse Dynamik mit Newton-Euler und Jacobi-Matrix mit geometrischer Berechnung).
if [ "$robot_kinconstr_exist" == "1" ] || [ "$robot_NQJ" != "$robot_NJ" ]; then
  blacklist="$blacklist
  gravload
  inertia
  invdyn_floatb_eulxyz_nnew
  robot_jacobig_cutforce_mdh_num
  robot_jacobig_mdh_eulxyz_num
  robot_jacobig_mdh_num
  robot_jacobigD_mdh_eulxyz_num
  robot_jacobigD_mdh_num
  "
fi
# Falls die Minimalparameterform nicht existiert, funktionieren die Trajektorien-Funktionen für Regressorform nicht
if [ $robot_NMPVFIXB == "NOTDEFINED" ]; then
  blacklist="$blacklist
  invdynJ_fixb_mdp_slag_vp_traj
  invdynJ_fixb_mdp_slag_vr_traj
  invdynJ_fixb_regmin_slag_vp_traj
  "
fi

# Falls nur die Kinematik generiert werden soll, liegen teilweise nicht genug Informationen vor.
if [ "$kinematicsOnly" == 1 ]; then
  blacklist="$blacklist
  gravload
  inertia
  invdyn
  "
fi;

# Vorlagen aus Robotik-Repo kopieren (falls verfügbar)
# Benutze eine separate Datei zum Zeigen auf den Pfad, falls unter Windows.
linkfiles="$repo_pfad/robotics_repo_path_linux
  $repo_pfad/robotics_repo_path"
for linkfile in $linkfiles; do
if [ -f $linkfile ]; then
  robrepopath=`sed -n -e 's/^robotics_repo_path := "\(.*\)":[\r]*/\1/p' $linkfile`
  if [ "$robrepopath" == "" ]; then
    echo "Ausdruck "robotics_repo_path= \"/pfad/zum/repo\":" in $linkfile nicht gefunden"
    continue
  fi
  if [ -d $robrepopath ]; then
    for f in `find "$robrepopath/kinematics" -name 'robot_*.template'`; do
      filename="${f##*/}"
      if [[ -L $template_pfad/$filename ]]; then
        continue # Symbolischen Link erhalten (sonst Kopierfehler)
      fi
      cp $f $template_pfad/$filename
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
  
  # Prüfe, ob die Datei für dieses System nicht generiert werden sollte:
  donotgenerate=0
  for blstr in $blacklist; do
    if [[ $f == *"$blstr"* ]]; then
      donotgenerate=1
      break
    fi
  done
  if [ "$donotgenerate" == "1" ]; then
    echo "Überspringe $filename wegen Filter $blstr"
    continue
  fi

  # Datei kopieren
  cp $dir/$filename $fcn_pfad/$filename_new

  # Platzhalter in Datei ersetzen
  source robot_codegen_matlabfcn_postprocess.sh $fcn_pfad/$filename_new 0
done

# Erzeuge Kinematik-Parameter-Funktion. Diese Funktion kann aufgerufen, um die strukturabhängigen
# Parameter des Roboters in einer kompilierbaren Funktion zu erhalten (Topologie)
zieldat=$fcn_pfad/${robot_name}_structural_kinematic_parameters.m
printf "\n%% Aus parameters_mdh_v_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name}/tmp/parameters_mdh_v_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "v_mdh = uint8($varname_tmp);" >> $zieldat
printf "\n%% Aus parameters_mdh_sigma_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name}/tmp/parameters_mdh_sigma_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "sigma_mdh = $varname_tmp;" >> $zieldat
printf "\n%% Aus parameters_mdh_mu_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name}/tmp/parameters_mdh_mu_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "mu_mdh = $varname_tmp;" >> $zieldat
printf "\n%% Aus Roboterdefinition\n" >> $zieldat
echo "% Anzahl der Robotersegmente (inkl Basis)" >> $zieldat
echo "NL = $robot_NL;" >> $zieldat
echo "% Anzahl der Kinematikparameter" >> $zieldat
echo "NKP = $robot_NKP;" >> $zieldat
echo "% Anzahl der Minimalkoordinaten (für hybride Systeme)" >> $zieldat
echo "NQJ = $robot_NQJ;" >> $zieldat
echo "% Namen der Kinematikparameter" >> $zieldat
printf "pkin_names = {" >> $zieldat
i=0
for Kp in $robot_KP; do
  i=$((i+1)); 
  printf "\'$Kp\'" >> $zieldat
  if [ "$i" -lt "$robot_NKP" ]; then
    printf ", " >> $zieldat
  fi
done
printf "};\n" >> $zieldat

# Parameter-Funktion, die die MDH-Parameter für einen gegebenen Parametervektor pkin ausgibt
zieldat=$fcn_pfad/${robot_name}_pkin2mdhparam.m
quelldat_tmp=$repo_pfad/codeexport/${robot_name}/tmp/parameters_mdh_all
echo "" > $quelldat_tmp
p="beta b alpha a theta d qoffset"
for exp in ${p[@]}; do
  printf "\n%% Aus parameters_mdh_${exp}_matlab.m\n" >> $quelldat_tmp
  cat $repo_pfad/codeexport/${robot_name}/tmp/parameters_mdh_${exp}_matlab.m >> $quelldat_tmp
  varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $quelldat_tmp`
  echo "${exp}_mdh = $varname_tmp;" >> $quelldat_tmp
done
cat $tmp_pfad/robot_matlabtmp_par_KP.m > ${quelldat_tmp}.subsvar # damit werden die Elemente von pkin automatisch in der Funktion mit dem indizierten Vektor `pkin`ersetzt.
cat $quelldat_tmp >> $zieldat
source robot_codegen_matlabfcn_postprocess.sh $zieldat 0 0 ${quelldat_tmp}.subsvar

# Parameter-Funktion, die den Parametervektor pkin für gegebene MDH-Parameter ausgibt
zieldat=$fcn_pfad/${robot_name}_mdhparam2pkin.m
printf "\n%% Aus parameter_kin_from_mdh_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name}/tmp/parameter_kin_from_mdh_matlab.m >> $zieldat
source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
if [ "$robot_KP" == "dummy" ]; then
  printf "pkin = NaN;%% Dummy-Wert, da pkin nicht leer sein soll\n" >> $zieldat
fi

# Erzeuge Dynamik-Parameter-Funktion. Damit werden benutzerspezifische Dynamikparameter
# für die Testskripte erzeugt (z.B. wenn einzelne Parameter Null gesetzt werden)
zieldat=$fcn_pfad/${robot_name}_dynamics_parameters_modification.m # Die Dateikopf-Vorlage wird automatisch hier angelegt
zieldat2=$test_pfad/${robot_name}_dynamics_parameters_modification.m # Die Datei soll am Ende hier liegen
printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat

printf "\n%%%% Variable Initialization" >> $zieldat
printf "\n%% Complete set of dynamics parameters that can be reduced by the user." >> $zieldat
cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
cat $tmp_pfad/robot_matlabtmp_par_m.m >> $zieldat
cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> $zieldat
cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> $zieldat

printf "\n%%%% Parameter Postprocessing / Set Output" >> $zieldat
printf "\n%% Create the reduced set of dynamics parameters that is also used for generation of the dynamics equations." >> $zieldat

printf "\n%% Aus parameters_dyn_mges_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name}/tmp/parameters_dyn_mges_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "m = $varname_tmp;" >> $zieldat

printf "\n%% Aus parameters_dyn_rSges_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name}/tmp/parameters_dyn_rSges_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "rSges = $varname_tmp;" >> $zieldat

printf "\n%% Aus parameters_dyn_Icges_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name}/tmp/parameters_dyn_Icges_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "Icges = $varname_tmp;" >> $zieldat

# Platzhalter ersetzen
source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
# Funktion in Ordner für Testskripte verschieben (wird nur dafür benötigt)
mv $zieldat $zieldat2
