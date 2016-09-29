#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Kinematik"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/robot_codegen_scripts/tmp
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Direkte Kinematik (Ohne Basis-Orientierung oder Höhe)
quelldat=$repo_pfad/codeexport/${robot_name}_fkine_mdh_floatb_twist_rotmat_matlab.m
zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_fkine_fixb_rotmat_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat ${tmp_pfad}_head/robot_matlabtmp_fkine_fixb_rotmat.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KCP.m >> $zieldat
  echo "%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
  printf "rxs_base=0;\nrys_base=0;\nrzs_base=0;\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_KCP.m >> $zieldat
  printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
  varname_tmp=`grep "=" $zieldat| tail -1 | sed 's/\(.*\)=.*/\1/'`
  echo "T_ges = $varname_tmp;" >> $zieldat
  echo "%% Postprocessing: Reshape Output" >> $zieldat
  printf "%% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)\n" >> $zieldat
  printf "T_c_mdh = NaN(4,4,%%NL%%);\nfor i = 1:%%NL%%\n  T_c_mdh(:,:,i) = T_ges((i-1)*4+1 : 4*i, :);\nend\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Direkte Kinematik (Basis-Orientierung (EulerXYZ) und Position)
quelldat=$repo_pfad/codeexport/${robot_name}_fkine_mdh_floatb_eulangrpy_rotmat_matlab.m
zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_fkine_floatb_eulangrpy_rotmat_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat ${tmp_pfad}_head/robot_matlabtmp_fkine_floatb_eulangrpy_rotmat.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_rB.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KCP.m >> $zieldat
  echo "%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_rB.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_KCP.m >> $zieldat
  printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
  varname_tmp=`grep "=" $zieldat| tail -1 | sed 's/\(.*\)=.*/\1/'`
  echo "T_ges = $varname_tmp;" >> $zieldat
  echo "%% Postprocessing: Reshape Output" >> $zieldat
  printf "%% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)\n" >> $zieldat
  printf "T_c_mdh = NaN(4,4,%%NL%%);\nfor i = 1:%%NL%%\n  T_c_mdh(:,:,i) = T_ges((i-1)*4+1 : 4*i, :);\nend\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Gelenk-Transformationsmatrizen
quelldat=$repo_pfad/codeexport/${robot_name}_joint_transformation_mdh_rotmat_matlab.m
zieldat=$repo_pfad/codeexport/matlabfcn/${robot_name}_joint_trafo_rotmat_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat ${tmp_pfad}_head/robot_matlabtmp_joint_transformation.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_mdh.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KCP.m >> $zieldat
  echo "%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_q.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_mdh.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_KCP.m >> $zieldat
  printf "\n%%%% Symbolic Calculation\n%%From ${quelldat##*/}\n" >> $zieldat
  cat $quelldat >> $zieldat
  # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
  varname_tmp=`grep "=" $zieldat| tail -1 | sed 's/\(.*\)=.*/\1/'`
  echo "T_ges = $varname_tmp;" >> $zieldat
  echo "%% Postprocessing: Reshape Output" >> $zieldat
  printf "%% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)\n" >> $zieldat
  printf "T_mdh = NaN(4,4,%%NJ%%);\nfor i = 1:%%NJ%%\n  T_mdh(:,:,i) = T_ges((i-1)*4+1 : 4*i, :);\nend\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi
