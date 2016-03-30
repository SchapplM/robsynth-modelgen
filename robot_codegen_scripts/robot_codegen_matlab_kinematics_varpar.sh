#!/bin/bash
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/robot_codegen_scripts/tmp/
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Direkte Kinematik
quelldat=$repo_pfad/codeexport/${robot_name}_fkine_rotmat_matlab.m
zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_fkine_rotmat_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat $tmp_pfad/robot_matlabtmp_fkine_rotmat.head.m > $zieldat
  printf "%%%%Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
  echo "%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
  printf "rxs_base=0;\nrys_base=0;\nrzs_base=0;\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
  printf "\n%%%%Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
  varname_tmp=`grep "=" $zieldat| tail -1 | sed 's/\(.*\)=.*/\1/'`
  echo "T_ges = $varname_tmp;" >> $zieldat
  echo "%% Postprocessing: Reshape Output" >> $zieldat
  printf "%% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)\n" >> $zieldat
  printf "T_c_mdh = NaN(4,4,%%NL%%);\nfor i = 1:%%NL%%\n  T_c_mdh(:,:,i) = T_ges((i-1)*4+1 : 4*i, :);\nend\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat
  # letzte Zeile wieder entfernen (fehlerhaft, da robot_codegen_matlabfcn_postprocess.sh hier nicht passend)
  head -n -1 $zieldat > $tmp_pfad/temp.txt
  mv $tmp_pfad/temp.txt $zieldat
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi


