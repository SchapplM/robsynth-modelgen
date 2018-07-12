#!/bin/bash -e
# Starte die Code-Generierung für eine repräsentative Auswahl der Beispiele
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
# (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

examplelist="robot_env_RPR1.example
robot_env_prothese2dof.example
robot_env_scara3dof.example
robot_env_scara4dof.example
robot_env_imesthor.example"

for ex in $examplelist; do
  echo "Starte Beispiel $ex"
  cp robot_codegen_definitions/examples/$ex robot_codegen_definitions/robot_env
  ./robot_codegen_start.sh "$@"
done
