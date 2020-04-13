#!/bin/bash -e
# Erstelle temporäre Variablen mit Code-Schnipseln, die für die Erzeugung von Matlab-Code benötigt wird.
# Code-Schnipsel enthalten assert-Befehle
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-04
# (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

repo_pfad=$(pwd)/../
tmp_pfad=$repo_pfad/workdir/tmp
source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh

# Schnipsel für Matlab-varpar-Dateien vorbereiten
# Kinematikparameter (überschreibe Zuweisung von System ohne IC)
echo "assert(isreal(pkin) && all(size(pkin) == [$robot_NKP 1]), ...
  '%FN%: pkin has to be [${robot_NKP}x1] (double)');" > $tmp_pfad/robot_matlabtmp_assert_KP.m
