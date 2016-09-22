#!/bin/bash -e
# Passe die Vorlagen für Dynamik-Testfunktionen an den Roboter an.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Testskripte für Matlabfunktionen"

repo_pfad=$(pwd)/..
testfcn_pfad=$repo_pfad/robot_codegen_testfunctions
tmp_pfad=$repo_pfad/robot_codegen_scripts/tmp

# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

dateiliste_testfunction="
  robot_varpar_kinematics_test.m.template
  robot_varpar_invdyn_test.m.template
  robot_varpar_floatbase_test.m.template
  robot_varpar_paramlin_test.m.template
  robot_varpar_simulink_test.m.template
  robot_varpar_fixbase_num_test.m.template
  robot_varpar_floatbase_num_test.m.template
  robot_varpar_testfunctions_parameter.m.template
  robot_test_everything.m.template
  simulink/lib_robot_dynamics.mdl.template
  simulink/robot_fdyn_fixb_test_mp_start.m.template
  simulink/robot_fdyn_fixb_test_mp_vp.mdl.template
  simulink/robot_fdyn_fixb_test_settings_default.m.template
  simulink/robot_fdyn_fixb_test_vp1.mdl.template
  simulink/robot_fdyn_fixb_test_vp1_start.m.template
  simulink/robot_fdyn_floatb_eulangrpy_test.mdl.template
  simulink/robot_fdyn_floatb_eulangrpy_test_settings_default.m.template
  simulink/robot_fdyn_floatb_eulangrpy_test_start.m.template
"
for dat in $dateiliste_testfunction
do
  tmpdat_full=$testfcn_pfad/$dat
  filename="${tmpdat_full##*/}"                      # Strip longest match of */ from start
  dir="${tmpdat_full:0:${#tmpdat_full} - ${#filename} - 1}" # Substring from 0 thru pos of filename

  # Neuen Dateinamen generieren
  tmp="${filename/robot/$robot_name}"
  filename_new="${tmp/.template/}"
  
  # Datei kopieren
  cp $dir/$filename $dir/$filename_new

  # Platzhalter in Datei ersetzen
  source robot_codegen_matlabfcn_postprocess.sh $dir/$filename_new 0
done


# Parameter-Generierungsskript anpassen (damit nicht MDH-Parameter mit Zufallswerten belegt sind, die einen bestimmten Wert haben sollen)
zieldat=$testfcn_pfad/${robot_name}_varpar_testfunctions_parameter.m
cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
# Hänge alle Ausdrücke für die MDH-Parameter an und ersetze die Ergebnisvariable
# So werden nur die Bestandteile übernommen, die Werte enthalten
cat $repo_pfad/codeexport/${robot_name}_parameters_mdh_d.m >> $zieldat
varname_tmp=`grep "=" $zieldat | tail -1 | sed 's/\(.*\)=.*/\1/'`
echo "d_mdh = $varname_tmp;" >> $zieldat
cat $repo_pfad/codeexport/${robot_name}_parameters_mdh_a.m >> $zieldat
varname_tmp=`grep "=" $zieldat | tail -1 | sed 's/\(.*\)=.*/\1/'`
echo "a_mdh = $varname_tmp;" >> $zieldat
cat $repo_pfad/codeexport/${robot_name}_parameters_mdh_b.m >> $zieldat
varname_tmp=`grep "=" $zieldat | tail -1 | sed 's/\(.*\)=.*/\1/'`
echo "b_mdh = $varname_tmp;" >> $zieldat
cat $repo_pfad/codeexport/${robot_name}_parameters_mdh_beta.m >> $zieldat
varname_tmp=`grep "=" $zieldat | tail -1 | sed 's/\(.*\)=.*/\1/'`
echo "beta_mdh = $varname_tmp;" >> $zieldat
cat $repo_pfad/codeexport/${robot_name}_parameters_mdh_alpha.m >> $zieldat
varname_tmp=`grep "=" $zieldat | tail -1 | sed 's/\(.*\)=.*/\1/'`
echo "alpha_mdh = $varname_tmp;" >> $zieldat
cat $repo_pfad/codeexport/${robot_name}_parameters_mdh_qoffset.m >> $zieldat
varname_tmp=`grep "=" $zieldat | tail -1 | sed 's/\(.*\)=.*/\1/'`
echo "q_offset_mdh = $varname_tmp;" >> $zieldat
cat $repo_pfad/codeexport/${robot_name}_parameters_mdh_v.m >> $zieldat
varname_tmp=`grep "=" $zieldat | tail -1 | sed 's/\(.*\)=.*/\1/'`
echo "v_mdh = uint8($varname_tmp);" >> $zieldat
# Ersetze "_mdh", damit die Variablennamen stimmen
sed -i "s/_mdh//g" $zieldat

