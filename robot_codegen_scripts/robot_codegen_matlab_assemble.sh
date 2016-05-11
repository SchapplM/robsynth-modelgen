#!/bin/bash 
# Zusammensetzen von Teilausdrücken zu einem vollständigem Ausdruck.
#
# Beim Maple-Code-Export ist es teilweise schneller, größere Ausdrücke (z.B. 30x1-Coriolisvektor)
# in einzelne 1x1-Einträge aufzuteilen und den Code zu optimieren, als den vollständigen Ausdruck
# zu optimieren.
# Die einzelnen Dateien für 1x1-Einträge werden hier wieder zusammengefasst.

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover


echo "Setze Teilausdrücke des exportierten Codes zu vollständigen Ausdrücken zusammen"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/robot_codegen_scripts/tmp
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

basemethodenames=( twist eulangrpy )

# Erstelle Matlab-Funktionen der explizit ausgerechneten Dynamik (nicht in Regressorform)
for (( dynpar=1; dynpar<=2; dynpar++ ))
do
  for basemeth in "${basemethodenames[@]}"
  do
    # Coriolisvektor (Floating Base, Basis)
    zieldat=$repo_pfad/codeexport/${robot_name}_coriolisvec_base_floatb_${basemeth}_par${dynpar}_matlab.m
    if ! [ -f $zieldat ]; then
      # Prüfe, ob alle Einzel-Dateien vorhanden sind
      vollst=1
      for (( i=1; i<=6; i++ ))
      do
        teildat=$repo_pfad/codeexport/${robot_name}_coriolisvec_floatb_${basemeth}_${i}_par${dynpar}_matlab.m
        if ! [ -f $teildat ]; then
          vollst=0
          echo "Code in ${teildat##*/} nicht vorhanden."
        fi
      done
      # echo 
      # Setze den vollständigen Ausdruck aus Teil-Dateien zusammen
      if [ $vollst == 1 ]; then
        touch $zieldat
        for (( i=1; i<=5; i++ )); do
          teildat=$repo_pfad/codeexport/${robot_name}_coriolisvec_floatb_${basemeth}_${i}_par${dynpar}_matlab.m
          # prüfe, welches die Ausgabevariable des Maple-exportierten Codes ist
          # Nehme nur die ersten 50 Zeichen der letzten Zeile (falls der Code in einer Zeile steht).
          varname_tmp=`cut -c-50 $teildat | grep "=" | tail -1 | sed 's/\(.*\)=.*/\1/'`
          # Quelltext aus Teil-Datei hineinkopieren
          printf "\n\n%% from ${teildat##*/}\n" >> $zieldat
          cat $teildat >> $zieldat
          # Neue Variable zuweisen
          echo "c$i = $varname_tmp;" >> $zieldat
        done
        # Gesamt-Ausdruck bestimmen
        echo "cqb = [c1;c2;c3;c4;c5;c6];" >> $zieldat
        echo "${zieldat##*/} aus Teildateien erstellt"
      fi
    else
      echo "${zieldat##*/} existiert. Kein Zusammensetzen notwendig."
    fi

  done # floatb_twist/floatb_eulangrpy
done # par1/par2
