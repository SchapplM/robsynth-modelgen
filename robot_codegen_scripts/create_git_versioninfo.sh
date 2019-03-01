#!/bin/bash -e
# Erstelle Datei mit Git-Versionsinfo, die in die generierten Matlab-Funktionen eingefügt wird.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover


repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp

# Versionsinformationen als Datei erstellen. Der Inhalt dieser Datei wird dann in die Matlab-Funktionen eingefügt.
versionfile=$tmp_pfad/version_info.head.m
echo "% Quelle: HybrDyn-Toolbox" > $versionfile
now="$(date +'%Y-%m-%d %H:%M')"
printf "%% Datum: $now\n" >> $versionfile
rev=`git rev-parse HEAD`
revdatum=`git log -1 --date=short --pretty=format:%cd`
printf "%% Revision: $rev ($revdatum)\n" >> $versionfile
echo "% Moritz Schappler, moritz.schappler@imes.uni-hannover.de" >> $versionfile
echo "% (C) Institut für Mechatronische Systeme, Universität Hannover" >> $versionfile

