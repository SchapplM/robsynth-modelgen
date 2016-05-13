#!/bin/bash 
# Konvertiere ein Maple-Worksheet in eine mpl Datei
# Argument: Pfad zur ws-Datei

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

# Pfad zur mw-Datei
mw_file=$1

# Pfad zur mpl-Datei
# http://stackoverflow.com/questions/965053/extract-filename-and-extension-in-bash
fullpath=$1
filename="${fullpath##*/}"                      # Strip longest match of */ from start
dir="${fullpath:0:${#fullpath} - ${#filename}}" # Substring from 0 thru pos of filename
base="${filename%.[^.]*}"                       # Strip shortest match of . plus at least one non-dot char from end
mpl_file=$dir/$base.mpl

echo "Konvertiere $mw_file --> $mpl_file"

# Alte mpl-Datei löschen
rm -f $mpl_file

# Graphisches Maple starten (wird zum neu abspeichern benötigt)
cd /opt/maple18/bin/
./xmaple $mw_file &
# Warten bis geladen
sleep 7
# Speicher-Dialog öffnen
# Alt+F
xte 'keydown Alt_L'
sleep 1
xte 'keydown F' 
sleep 1
xte 'keyup F' 'keyup Alt_L'
# E    
sleep 1             
xte 'keydown E' 'keyup E'
sleep 1

# Anhängen einer Endung
# xte 'keydown End' 'keyup End'
#sleep 1
#xte 'keydown 0x05f' 'keyup 0x05f'
#xte 'keydown m' 'keyup m'
#xte 'keydown p' 'keyup p'
#xte 'keydown l' 'keyup l'

# Endung auswählen
sleep 1
xte 'keydown Tab' 'keyup Tab'
xte 'keydown Down' 'keyup Down'
xte 'keydown Down' 'keyup Down'
xte 'keydown Down' 'keyup Down'
xte 'keydown Down' 'keyup Down'
xte 'keydown Return' 'keyup Return'
sleep 1
xte 'keydown Tab' 'keyup Tab'
xte 'keydown Return' 'keyup Return'

#Beenden
sleep 1
xte 'keydown Alt_L'
sleep 1
xte 'keydown F' 
sleep 1
xte 'keyup F' 'keyup Alt_L'
sleep 1             
xte 'keydown X' 'keyup X'
sleep 1



