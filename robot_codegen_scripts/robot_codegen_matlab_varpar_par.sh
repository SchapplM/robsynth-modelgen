#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erledigt alle Schritte, nachdem der Code in Maple exportiert wurde
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Tim Job (Studienarbeit bei Moritz Schappler), 2018-12
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de
# (C) Institut für Mechatronische Systeme, Universität Hannover

echo "Generiere Matlabfunktionen für parallelen Roboter"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
# Initialisiere Variablen
source robot_codegen_tmpvar_bash_par.sh
source $repo_pfad/robot_codegen_definitions/robot_env_par.sh

# Erstelle Matlab-Hilfsdateien
source robot_codegen_tmpvar_matlab_par.sh
source robot_codegen_assert_matlab_par.sh
# Hilfsskript für Erzeugung symmetrischer Matrizen. Hier (noch) nicht benötigt.
#source robot_codegen_matlab_preparation.sh

# Korrigiere mit Maple generierte Matlab-Code-Dateien
./robot_codegen_matlabcode_postprocess_recursive.sh $repo_pfad/codeexport/$robot_name/tmp

# Erstelle Matlab-Funktionen der Kinematik
./robot_codegen_matlab_kinematics_parallel_varpar.sh

# Erstelle Matlab-Funktionen der explizit ausgerechneten Dynamik (nicht in Regressorform)
./robot_codegen_matlab_dynamics_parallel_varpar.sh

# Erstelle Matlab-Funktionen der parameterlinearen Dynamik
./robot_codegen_matlab_paramlin_parallel_varpar.sh



