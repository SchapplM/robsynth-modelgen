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

# Erstelle Liste mit nicht zu generierenden Dateien
blacklist=""
# Für Systeme mit kinematischen Zwangsbedingungen funktionieren einige Ansätze der nicht
# (z.B. Inverse Dynamik mit Newton-Euler und Jacobi-Matrix mit geometrischer Berechnung).
if [ "$robot_kinconstr_exist" == "1" ] || [ "$robot_NQJ" != "$robot_NJ" ]; then
  blacklist="$blacklist
  gravload
  inertia
  invdyn
  jacobi
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
    continue
  fi

  # Datei kopieren
  cp $dir/$filename $fcn_pfad/$filename_new

  # Platzhalter in Datei ersetzen
  source robot_codegen_matlabfcn_postprocess.sh $fcn_pfad/$filename_new 0
done

# Erzeuge Parameter-Funktion. Diese Funktion kann aufgerufen, um die strukturabhängigen
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

