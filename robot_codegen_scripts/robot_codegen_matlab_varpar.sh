#!/bin/bash
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erledigt alle Schritte, nachdem der Code in Maple exportiert wurde
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/robot_codegen_scripts/tmp/
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Erstelle Matlab-Hilfsdateien
source robot_codegen_tmpvar_matlab.sh
source robot_codegen_assert_matlab.sh

# Setze Teilausdrücke zu kompletten Ausdrücken zusammen
source robot_codegen_matlab_assemble.sh

# Erstelle Matlab-Funktionen der Kinematik
source robot_codegen_matlab_kinematics_varpar.sh

# Erstelle Matlab-Funktionen der explizit ausgerechneten Dynamik (nicht in Regressorform)
source robot_codegen_matlab_dynamics_fixb_varpar.sh
source robot_codegen_matlab_dynamics_floatb_varpar.sh

# Erstelle Matlab-Funktionen der parameterlinearen Dynamik
source robot_codegen_matlab_paramlin_varpar.sh

# Erstelle Matlab-Funktionen aus numerischer Berechnung
source robot_codegen_matlab_num_varpar.sh
