#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erledigt alle Schritte, nachdem der Code in Maple exportiert wurde
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp/
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Erstelle Matlab-Hilfsdateien
source robot_codegen_tmpvar_matlab.sh
source robot_codegen_assert_matlab.sh
source robot_codegen_matlab_preparation.sh

# Korrigiere mit Maple generierte Matlab-Code-Dateien
./robot_codegen_matlabcode_postprocess_recursive.sh $repo_pfad/codeexport/$robot_name/tmp

# Setze Teilausdrücke zu kompletten Ausdrücken zusammen
./robot_codegen_matlab_assemble.sh

# Erstelle Matlab-Funktionen der Kinematik
if [ "$parallel_robot" == "0" ]; then
./robot_codegen_matlab_kinematics_varpar.sh
fi;

# Erstelle Matlab-Funktionen der explizit ausgerechneten Dynamik (nicht in Regressorform)
if [ "$parallel_robot" == "0" ]; then
	./robot_codegen_matlab_dynamics_fixb_varpar.sh
	./robot_codegen_matlab_dynamics_floatb_varpar.sh
fi;
./robot_codegen_matlab_dynamics_parallel_varpar.sh

# Erstelle Matlab-Funktionen der parameterlinearen Dynamik
if [ "$parallel_robot" == "0" ]; then
	./robot_codegen_matlab_paramlin_fixb_varpar.sh
	./robot_codegen_matlab_paramlin_floatb_varpar.sh
fi;

# Erstelle Matlab-Funktionen aus numerischer Berechnung
if [ "$parallel_robot" == "0" ]; then
	./robot_codegen_matlab_num_varpar.sh
fi;

# Erstelle Matlab-Funktionen aus selbst definierten Dateien
addgenscript=$repo_pfad/robot_codegen_additional/scripts/${robot_name}_codegen_matlab_additional_varpar.sh
if [ -f $addgenscript ]; then
  # cd $repo_pfad/robot_codegen_additional/scripts/
  source $addgenscript
fi;

