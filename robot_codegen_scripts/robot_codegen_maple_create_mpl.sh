#!/bin/bash -e
# Erzeuge Maple mpl-Dateien aus den vorhanden Maple-Arbeitsblättern
# Dieser Vorgang muss nur bei Änderung eines Arbeitsblatts ausgeführt werden.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt
# Erfordert installierte Pakete: wmctrl

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/../
echo $repo_pfad
# rekursiv nach mw-Dateien suchen
find $repo_pfad -name '*.mw' -exec ../scripts/maple_convert_mw_mpl.sh {} \;

