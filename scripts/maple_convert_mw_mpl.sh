#!/bin/bash 
# Konvertiere ein Maple-Worksheet in eine mpl Datei
# Argument: Pfad zur ws-Datei
# Abhängigkeiten (Ubuntu): xautomation, xbindkeys

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

# Pfad zur mw-Datei
mw_file=$1

# Pfad zur mpl-Datei
# http://stackoverflow.com/questions/965053/extract-filename-and-extension-in-bash
fullpath=$1
filename="${fullpath##*/}"                      # Strip longest match of */ from start
dir="${fullpath:0:${#fullpath} - ${#filename} - 1}" # Substring from 0 thru pos of filename
base="${filename%.[^.]*}"                       # Strip shortest match of . plus at least one non-dot char from end
mpl_file=$dir/$base.mpl

echo "Konvertiere $mw_file --> $mpl_file"

# direktes einfügen des Dateipfades in Maple geht nicht.
# Starte gedit und kopiere den Text in die Zwischenablage
gedit &
wmctrl -a gedit
sleep 1
xte "str $mpl_file"
# markieren
xte 'keydown Shift_L' 'keydown Control_L'
sleep 1
xte 'key Home'
xte 'keyup Shift_L' 'keyup Control_L'
sleep 1
# kopieren
xte 'keydown Control_L'
sleep 1
xte 'keydown C' 
sleep 1
xte 'keyup C' 
sleep 1
xte 'keyup Control_L'
sleep 1

# Alte mpl-Datei löschen (Überschreiben-Dialog in Maple nicht möglich, da Befehle blind eingegeben werden und die Datei eventuell nicht existiert).
rm -f $mpl_file

# Graphisches Maple starten (wird zum neu abspeichern benötigt)
cd /opt/maple2019/bin/
./xmaple $mw_file &
# Warten bis geladen
sleep 10
# Speicher-Dialog öffnen
# Alt+F
sleep 2
xte 'keydown Alt_L'
sleep 2
xte 'keydown F' 'keyup F' 
sleep 1
xte 'keyup Alt_L'
# E    
sleep 1             
xte 'key E'
sleep 1

# Dateipfad vollständig eingeben.
# Sonst wird immer in den Ordner gespeichert, in den zuletzt eine Datei gespeichert wurde.

# Dateinamen einfügen
xte 'keydown Control_L'
sleep 1
xte 'keydown V' 
xte 'keyup V' 'keyup Control_L'

# Endung auswählen (Reihenfolge ist abhängig von Maple-Version. Hier für Maple 2019).
sleep 1
xte 'key Tab'
xte 'key Down'
xte 'key Down'
xte 'key Down'
xte 'key Down'
xte 'key Down'
xte 'key Down'
xte 'key Return'
sleep 1
xte 'key Tab'
xte 'key Return'

#Beenden
sleep 2
xte 'keydown Alt_L'
sleep 2
xte 'keydown F' 'keyup F' 
sleep 1
xte 'keyup Alt_L'
sleep 1             
xte 'key X'
sleep 2

