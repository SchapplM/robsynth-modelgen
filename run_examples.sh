#!/bin/bash -ex
# Starte die Code-Generierung für eine repräsentative Auswahl der Beispiele
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-07
# (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

# Eine Liste einfacher serieller Beispielsysteme starten
examplelist="
robot_env_prothese2dof
robot_env_scara3dof
robot_env_scara4dof
robot_env_imesthor
robot_env_kuka6dof_reddp"
for ex in $examplelist; do
  echo "Starte Beispiel $ex"
  cp robot_codegen_definitions/examples/${ex}.example robot_codegen_definitions/robot_env
  rm -rf codeexport/${ex:10}
  ./robot_codegen_start.sh "$@" --fixb_only
done

# Einfache Floating-Base Systeme berechnen
examplelist="
robot_env_prothese2dof"
for ex in $examplelist; do
  echo "Starte Beispiel $ex"
  cp robot_codegen_definitions/examples/${ex}.example robot_codegen_definitions/robot_env
  rm -rf codeexport/${ex:10}
  ./robot_codegen_start.sh "$@"
done

# Beispiel-Systeme für Parallele Roboter
examplelist="
robot_env_S_RRR|robot_env_P_3RRR
robot_env_S_RPR|robot_env_P_3RPR
robot_env_S_RUU|robot_env_P_Delta"
for ex in $examplelist; do
  echo "Starte Beispiel $ex"
  serenv=$(echo $ex | cut -f1 -d\|)
  parenv=$(echo $ex | cut -f2 -d\|)
  cp robot_codegen_definitions/examples/${serenv}.example robot_codegen_definitions/robot_env
  cp robot_codegen_definitions/examples/${parenv}.example robot_codegen_definitions/robot_env_par
  rm -rf codeexport/${serenv:10}
  rm -rf codeexport/${parenv:10}
  ./robot_codegen_start.sh "$@" --fixb_only --parrob
done

# Beispiel-Systeme (seriell) mit einfacher Gravitationskomponente
examplelist="
robot_env_prothese2dof
robot_env_scara3dof
robot_env_S_RPR"
for ex in $examplelist; do
  echo "Starte Beispiel $ex"
  cp robot_codegen_definitions/examples/${ex}.example robot_codegen_definitions/robot_env
  echo "g_world := <0;0;g3>:" >> robot_codegen_definitions/robot_env
  rm -rf codeexport/${ex:10}
  ./robot_codegen_start.sh "$@" --fixb_only
done

# Beispiel-Systeme (PKM) mit einfacher Gravitationskomponente
examplelist="
robot_env_S_RPR|robot_env_P_3RPR
robot_env_S_RRR|robot_env_P_3RRR
robot_env_S_RUU|robot_env_P_Delta"
for ex in $examplelist; do
  echo "Starte Beispiel $ex"
  serenv=$(echo $ex | cut -f1 -d\|)
  parenv=$(echo $ex | cut -f2 -d\|)
  cp robot_codegen_definitions/examples/${serenv}.example robot_codegen_definitions/robot_env
  cp robot_codegen_definitions/examples/${parenv}.example robot_codegen_definitions/robot_env_par
  echo "g_world := <0;0;g3>:" >> robot_codegen_definitions/robot_env
  echo "g_world := <0;0;g3>:" >> robot_codegen_definitions/robot_env_par
  rm -rf codeexport/${serenv:10}
  rm -rf codeexport/${parenv:10}
  ./robot_codegen_start.sh "$@" --fixb_only --parrob
done

# Für einige komplizierte Systeme nur die Kinematik berechnen
examplelist="
robot_env_lbr4p.example
robot_env_atlas5arm.example
robot_env_atlas5wbody.example"
for ex in $examplelist; do
  echo "Starte Beispiel $ex"
  cp robot_codegen_definitions/examples/$ex robot_codegen_definitions/robot_env
  rm -rf codeexport/${ex:10}
  ./robot_codegen_start.sh "$@" --fixb_only --kinematics_only
done

