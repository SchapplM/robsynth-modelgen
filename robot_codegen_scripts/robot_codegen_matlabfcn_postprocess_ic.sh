#!/bin/bash -e
# Nachbearbeitung einer automatisch erstellten Matlab-Funktion
# Es werden nur spezielle Namen aus den impliziten ZB ersetzt.
# Zusätzlich muss (danach) immer das gleiche Skript ohne "ic" aufgeführt werden.
#
# Argumente:
# 1: mfcndat: Pfad der zu bearbeitenden Datei
# 2: replacelastassignment: Falls 0 wird die letzte Variablenzuweisung in der Datei nicht geändert (für Skripte). Standard: 1
# 3: lastassignmentvector: Falls 1 wird die letzte Variablenzuweisung als Vektor (stehend) erzwungen. Der autogenerierte Maple-Code ist bei Vektoren manchmal liegend, manchmal stehend. Standard: 0
# 4: subsvardat: Matlab-Skript mit Ersetzungsausdrücken für die Maple-Variablen. Kann die ursprünglich vorgesehene Variableninitialisierung ersetzen. Standard: Leer
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover


repo_pfad=$(pwd)/..
tmp_head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
source $repo_pfad/robot_codegen_definitions/robot_env.sh
source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh

mfcndat=$1 # Dateipfad als Übergabeargument

# Setze Beschreibung der Eingabegrößen ein (diese sind für viele Funktionen identisch)
sed -i "/% %INPUT_PKIN%/r $tmp_head_pfad/robot_matlabtmp_comment_input_pkin.m" $mfcndat
sed -i "/%INPUT_PKIN%/d" $mfcndat

# Ersetze Platzhalterausdrücke $RN$, ...
# Hier müssen normale und nicht einfache Anführungszeichen für `sed` genommen werden. Sonst wird das $-Zeichen für die Variable als Text interpretiert...
sed -i "s/%RN%/$robot_name/g" $mfcndat
sed -i "s/%RNTE%/$robot_name_TE/g" $mfcndat
sed -i "s/%RNDE%/$robot_name_DE/g" $mfcndat
sed -i "s/%RNOL%/$robot_name_OL/g" $mfcndat
sed -i "s/%NKP%/$robot_NKP/g" $mfcndat
sed -i "s/%NKCP%/$robot_NKCP/g" $mfcndat
kpstring="pkin=[$(echo "$robot_KP" | sed "s/ /,/g")]';"
sed -i "s/%KPDEF%/$kpstring/g" $mfcndat
sed -i "s/%robot_NTAUJFIXBREGNN%/$robot_NTAUJFIXBREGNN/g" $mfcndat


