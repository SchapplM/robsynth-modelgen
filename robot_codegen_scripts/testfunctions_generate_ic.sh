#!/bin/bash -e
# Passe die Vorlagen für Dynamik-Testfunktionen an den Roboter an.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Tim Job (Studienarbeit bei Moritz Schappler), 2018-12
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover

echo "Generiere Testskript (IC) für Matlabfunktionen"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp

# Initialisiere Variablen
source robot_codegen_tmpvar_bash_ic.sh
source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh

testfcn_pfad=$repo_pfad/codeexport/$robot_name/testfcn

for f in $(find $repo_pfad/robot_codegen_testfunctions/ic -name "*.template")
do
  tmpdat_full=$f
  filename="${tmpdat_full##*/}"                      # Strip longest match of */ from start
  # Verzeichnis der Vorlagen-Datei
  dir1="${tmpdat_full:0:${#tmpdat_full} - ${#filename} - 1}" # Substring from 0 thru pos of filename

  # Neuen Dateinamen generieren
  tmp="${filename/robot/$robot_name}" # Ersetze Begriff robot mit Roboternamen
  filename_new="${tmp/.template/}" # Endung .template entfernen
  
  # Neues Verzeichnis generieren: Jetzt im Ordner codeexport/%RN%/testfcn
  dir2=$testfcn_pfad

  # Datei kopieren
  mkdir -p "$dir2"
  cp $dir1/$filename $dir2/$filename_new

  # Platzhalter in Datei ersetzen
  source robot_codegen_matlabfcn_postprocess_ic.sh $dir2/$filename_new
  source robot_codegen_matlabfcn_postprocess.sh $dir2/$filename_new 0
  source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh
done

# Parameter-Generierungsskript anpassen
zieldat=$testfcn_pfad/${robot_name}_varpar_testfunctions_parameter.m

# Winkelgrenzen in Parameter-Skript berücksichtigen
qlim_dat=$repo_pfad/robot_codegen_constraints/${robot_name}_kinematic_constraints_matlab.m
if [ -f "$qlim_dat" ]; then
  # TODO: Zeilenumbrüche nach Einsetzen wiederherstellen, Einrückung schön machen.
  REPSTR=`tr "\n" " " < $qlim_dat`
else
  # nur zufällige Winkel
  REPSTR="q_min = -pi*ones(NQJ,1);q_max = pi*ones(NQJ,1);"
fi;
# Ersetzungsausdruck für Winkelausdruck in Vorlage ersetzen.
sed -i "s|%REPLACE_QDEF%|$REPSTR|" $zieldat

# Parameter-Generierungsskript anpassen (damit nicht MDH-Parameter mit Zufallswerten belegt sind, die einen bestimmten Wert haben sollen)
printf "\n\n%%%% MDH-Parametereinträge auf Zufallswerte setzen" >> $zieldat
printf "\n%% Aus robot_matlabtmp_par_mdh.m" >> $zieldat
cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat

# Werte für Kinematik-Parametervektor in Parameter-Generierungsskript eintragen
# Setzt auch Zahlenwerte für die Kinematikparameter der MDH-Notation, falls gegeben.
KCP_dat1=$repo_pfad/robot_codegen_constraints/${robot_name}_kinematic_parameter_values.m
if [ -f $KCP_dat1 ]; then
  printf "\n%%%% Werte für Kinematikparameter direkt eintragen\n" >> $zieldat
  printf "\n%% Aus ${robot_name}/kinematic_parameter_values.m\n" >> $zieldat
  cat $KCP_dat1 >> $zieldat
else
  # Zufällige Werte für Kinematikparameter
  printf "\n%%%% Zufällige Werte für Kinematikparameter\n" >> $zieldat
  for Kp in $robot_KP; do
    echo "$Kp = rand(1,1);" >> $zieldat
  done
fi
KP_dat2=$repo_pfad/codeexport/${robot_name}/tmp/parameter_kin_matlab.m
if [ -f "$KP_dat2" ]; then
  printf "\n%% Aus ${robot_name}/parameter_kin_matlab.m\n" >> $zieldat
  cat $KP_dat2 >> $zieldat
  varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
  echo "pkin = $varname_tmp;" >> $zieldat
  echo "if isempty(pkin)" >> $zieldat
  echo "  pkin = 0;%Platzhalter-Eingabe" >> $zieldat
  echo "end" >> $zieldat
else
  echo "Kinematik-Parametervektor in $KP_dat2 nicht gefunden"
  exit 1
fi;

KP_dat3=$repo_pfad/codeexport/${robot_name_OL}/tmp/parameter_kin_matlab.m
if [ -f "$KP_dat3" ]; then
  printf "\n%% Aus ${robot_name_OL}/parameter_kin_matlab.m\n" >> $zieldat
  cat $KP_dat3 >> $zieldat
  varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
  echo "pkinOL = $varname_tmp;" >> $zieldat
  echo "if isempty(pkinOL)" >> $zieldat
  echo "  pkinOL = 0;%Platzhalter-Eingabe" >> $zieldat
  echo "end" >> $zieldat
else
  echo "Kinematik-Parametervektor in $KP_dat3 nicht gefunden"
  exit 1
fi;

# Hänge alle Ausdrücke für die MDH-Parameter an und ersetze die Ergebnisvariable
# So werden nur die Bestandteile übernommen, die Werte enthalten
# Null-Einträge werden automatisch zu Null gesetzt.
printf "\n\n%%%% MDH-Parametereinträge mit vorgegebenen Werten überschreiben" >> $zieldat
printf "\n%% Aus ${robot_name_OL}/parameters_mdh_d_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name_OL}/tmp/parameters_mdh_d_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "d_mdh = $varname_tmp;" >> $zieldat
printf "\n%% Aus ${robot_name_OL}/parameters_mdh_a_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name_OL}/tmp/parameters_mdh_a_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "a_mdh = $varname_tmp;" >> $zieldat
printf "\n%% Aus ${robot_name_OL}/parameters_mdh_theta_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name_OL}/tmp/parameters_mdh_theta_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "theta_mdh = $varname_tmp;" >> $zieldat
printf "\n%% Aus ${robot_name_OL}/parameters_mdh_b_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name_OL}/tmp/parameters_mdh_b_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "b_mdh = $varname_tmp;" >> $zieldat
printf "\n%% Aus ${robot_name_OL}/parameters_mdh_beta_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name_OL}/tmp/parameters_mdh_beta_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "beta_mdh = $varname_tmp;" >> $zieldat
printf "\n%% Aus ${robot_name_OL}/parameters_mdh_alpha_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name_OL}/tmp/parameters_mdh_alpha_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "alpha_mdh = $varname_tmp;" >> $zieldat
printf "\n%% Aus ${robot_name_OL}/parameters_mdh_qoffset_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name_OL}/tmp/parameters_mdh_qoffset_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "q_offset_mdh = $varname_tmp;" >> $zieldat
printf "\n%% Aus ${robot_name_OL}/parameters_mdh_v_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name_OL}/tmp/parameters_mdh_v_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "v_mdh = uint8($varname_tmp);" >> $zieldat
printf "\n%% Aus ${robot_name_OL}/parameters_mdh_sigma_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name_OL}/tmp/parameters_mdh_sigma_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "sigma_mdh = $varname_tmp;" >> $zieldat
printf "\n%% Aus ${robot_name_OL}/parameters_mdh_mu_matlab.m\n" >> $zieldat
cat $repo_pfad/codeexport/${robot_name_OL}/tmp/parameters_mdh_mu_matlab.m >> $zieldat
varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
echo "mu_mdh = $varname_tmp;" >> $zieldat
# Ersetze "_mdh", damit die Variablennamen stimmen
sed -i "s/_mdh//g" $zieldat

if [ -f $repo_pfad/codeexport/${robot_name_OL}/tmp/kinconstr_index_dependant_joints_matlab.m ]; then
  printf "\n%% Aus ${robot_name_OL}/kinconstr_index_dependant_joints_matlab.m\n" >> $zieldat
  cat $repo_pfad/codeexport/${robot_name_OL}/tmp/kinconstr_index_dependant_joints_matlab.m >> $zieldat
  varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
  echo "Ind_depjoints = $varname_tmp;" >> $zieldat
else
  echo "Ind_depjoints = false(NJ,1);" >> $zieldat
fi

# Schreibe Ausgangsvariable (lese Teil aus Vorlage)
quelldat=$repo_pfad/robot_codegen_testfunctions/robot_varpar_testfunctions_parameter.m.template2
sed "s/%RN%_dynamics_parameters_modification(pkin/%RNOL%_dynamics_parameters_modification(pkinOL/g" $quelldat >> $zieldat

# Platzhalter (nochmal) in Datei ersetzen. Notwendig, weil auch Platzhalter in
# der template2-Datei sind, die nicht automatisch ersetzt wurden. 
source robot_codegen_matlabfcn_postprocess_ic.sh $zieldat 0
