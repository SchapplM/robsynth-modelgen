#!/bin/bash -e
# Gibt die letzte Variable (die zugewiesen wird) eines (Matlab)-Codes aus.
# Die Zuweisung muss in der letzten Zeile stehen
#
# Eingabe: Dateipfad

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-05
# (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

varname_tmp=`grep "=" $1 | tail -1 | sed 's/\([a-zA-Z0-9_]*\).*/\1/'`
echo $varname_tmp
