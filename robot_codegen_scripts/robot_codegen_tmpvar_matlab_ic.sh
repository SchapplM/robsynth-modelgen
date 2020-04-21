#!/bin/bash -e
# Erstelle temporäre Variablen mit Code-Schnipseln, die für die Erzeugung von Matlab-Code benötigt wird.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-04
# (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

repo_pfad=$(pwd)/../
tmp_pfad=$repo_pfad/workdir/tmp
source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh


# Kinematikparameter (alle als Vektor): MDH-Parameter und Parameter für (implizite) Zwangsbedingungen
# Das überschreibt die Kinematikparameter der offenen Struktur ohne implizite ZB.
echo "" > $tmp_pfad/robot_matlabtmp_par_KP.m
i=0;
for Kp in $robot_KP; do
  i=$((i+1));          
  echo "$Kp = pkin($i);" >> $tmp_pfad/robot_matlabtmp_par_KP.m
done
