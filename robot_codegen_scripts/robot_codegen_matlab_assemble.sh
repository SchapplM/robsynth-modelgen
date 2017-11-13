#!/bin/bash -e
# Zusammensetzen von Teilausdrücken zu einem vollständigem Ausdruck.
#
# Beim Maple-Code-Export ist es teilweise schneller, größere Ausdrücke (z.B. 30x1-Coriolisvektor)
# in einzelne 1x1-Einträge aufzuteilen und den Code zu optimieren, als den vollständigen Ausdruck
# zu optimieren.
# Die einzelnen Dateien für 1x1-Einträge werden hier wieder zusammengefasst.

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover


echo "Setze Teilausdrücke des exportierten Codes zu vollständigen Ausdrücken zusammen"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/robot_codegen_scripts/tmp
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

basemethodenames=( eulangrpy ) # Die zusammenzusetzenden Funktionen sind nicht für "twist" verfügbar

# Erstelle Matlab-Funktionen der explizit ausgerechneten Dynamik (nicht in Regressorform)
for (( dynpar=1; dynpar<=2; dynpar++ ))
do
  for basemeth in "${basemethodenames[@]}"
  do
    # Coriolisvektor (Floating Base, Basis)
    zieldat=$repo_pfad/codeexport/${robot_name}/tmp/coriolisvecB_floatb_${basemeth}_par${dynpar}_matlab.m
    if ! [ -f $zieldat ]; then
      echo "${zieldat##*/} existiert nicht. Versuche Zusammenzusetzen."
      # Prüfe, ob alle Einzel-Dateien vorhanden sind
      vollst=1
      for (( i=1; i<=6; i++ ))
      do
        teildat=$repo_pfad/codeexport/${robot_name}/tmp/coriolisvec_floatb_${basemeth}_${i}_par${dynpar}_matlab.m
        if ! [ -f $teildat ]; then
          vollst=0
          echo "Code in ${teildat##*/} nicht vorhanden."
          break
        fi
      done
      # Setze den vollständigen Ausdruck aus Teil-Dateien zusammen
      if [ $vollst == 1 ]; then
        touch $zieldat
        for (( i=1; i<=6; i++ )); do
          teildat=$repo_pfad/codeexport/${robot_name}/tmp/coriolisvec_floatb_${basemeth}_${i}_par${dynpar}_matlab.m
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
    
    # Coriolis-Matrix (Floating Base)
    zieldat=$repo_pfad/codeexport/${robot_name}/tmp/coriolismat_floatb_${basemeth}_par${dynpar}_matlab.m
    if ! [ -f $zieldat ]; then
      echo "${zieldat##*/} existiert nicht. Versuche Zusammenzusetzen."
      # Prüfe, ob alle Einzel-Dateien vorhanden sind
      vollst=1
      for (( i=1; i<=( $(($robot_NJ+6)) ); i++ )) do
      for (( j=1; j<=( $(($robot_NJ+6)) ); j++ )) do
        teildat=$repo_pfad/codeexport/${robot_name}/tmp/coriolismat_floatb_${basemeth}_${i}_${j}_par${dynpar}_matlab.m
        if ! [ -f $teildat ]; then
          vollst=0
          echo "Code in ${teildat##*/} nicht vorhanden."
          break 2
        fi
      done; done
      # Setze den vollständigen Ausdruck aus Teil-Dateien zusammen
      if [ $vollst == 1 ]; then
        touch $zieldat
        printf "\nCq = NaN(%d,%d);\n" "$(($robot_NJ+6))" "$(($robot_NJ+6))" >> $zieldat
        for (( i=1; i<=( $(($robot_NJ+6)) ); i++ )) do
        for (( j=1; j<=( $(($robot_NJ+6)) ); j++ )) do
          teildat=$repo_pfad/codeexport/${robot_name}/tmp/coriolismat_floatb_${basemeth}_${i}_${j}_par${dynpar}_matlab.m
          # prüfe, welches die Ausgabevariable des Maple-exportierten Codes ist
          # Nehme nur die ersten 50 Zeichen der letzten Zeile (falls der Code in einer Zeile steht).
          varname_tmp=`cut -c-50 $teildat | grep "=" | tail -1 | sed 's/\(.*\)=.*/\1/'`
          # Quelltext aus Teil-Datei hineinkopieren
          printf "\n\n%% from ${teildat##*/}\n" >> $zieldat
          cat $teildat >> $zieldat
          # Neue Variable zuweisen
          echo "Cq(${i},${j}) = $varname_tmp;" >> $zieldat
        done; done;
        echo "${zieldat##*/} aus Teildateien erstellt"
      fi    else
      echo "${zieldat##*/} existiert. Kein Zusammensetzen notwendig."
    fi
  done # floatb_twist/floatb_eulangrpy
done # par1/par2


# Erstelle Matlab-Funktionen der Jacobi-Matrizen
# TODO: Für andere Jacobi-Teilmatrizen auch
for (( jacart=1; jacart<=1; jacart++ ))
do
  for (( ib=1; ib<=$robot_NL; ib++ ))
  do
    zieldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobig_rot_${ib}_floatb_twist_matlab.m
    if ! [ -f $zieldat ]; then
      echo "${zieldat##*/} existiert nicht. Versuche Zusammenzusetzen."
      # Prüfe, ob alle Einzel-Dateien vorhanden sind
      vollst=1
      for (( i=1; i<=3; i++ )) do
      for (( j=1; j<=( $(($robot_NQJ)) ); j++ )) do
        teildat=$repo_pfad/codeexport/${robot_name}/tmp/jacobig_rot_${ib}_floatb_twist_${i}_${j}_matlab.m
        if ! [ -f $teildat ]; then
          vollst=0
          echo "Code in ${teildat##*/} nicht vorhanden."
          break 2
        fi
      done; done
      # Setze den vollständigen Ausdruck aus Teil-Dateien zusammen
      if [ $vollst == 1 ]; then
        touch $zieldat
        printf "\nJg_rot = NaN(%d,%d);\n" "$(($robot_NL))" "$(($robot_NQJ))" >> $zieldat
        for (( i=1; i<=3; i++ )) do
        for (( j=1; j<=( $(($robot_NQJ)) ); j++ )) do
          teildat=$repo_pfad/codeexport/${robot_name}/tmp/jacobig_rot_${ib}_floatb_twist_${i}_${j}_matlab.m
          # prüfe, welches die Ausgabevariable des Maple-exportierten Codes ist
          # Nehme nur die ersten 50 Zeichen der letzten Zeile (falls der Code in einer Zeile steht).
          varname_tmp=`cut -c-50 $teildat | grep "=" | tail -1 | sed 's/\(.*\)=.*/\1/'`
          # Quelltext aus Teil-Datei hineinkopieren
          printf "\n\n%% from ${teildat##*/}\n" >> $zieldat
          cat $teildat >> $zieldat
          # Neue Variable zuweisen
          echo "Jg_rot(${i},${j}) = $varname_tmp;" >> $zieldat
        done; done;
      fi
    else
      echo "${zieldat##*/} existiert. Kein Zusammensetzen notwendig."
    fi
  done
done
