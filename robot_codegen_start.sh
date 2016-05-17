#!/bin/bash 
# Starte die komplette Code-Erstellung für eine beliebige Roboterstrukter
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-05
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)

# Maple-Skripte starten
cd $repo_pfad/robot_codegen_scripts/
source $repo_pfad/robot_codegen_scripts/robot_codegen_maple_batch.sh

# Matlab-Funktionen generieren
cd $repo_pfad/robot_codegen_scripts/
source $repo_pfad/robot_codegen_scripts/robot_codegen_matlab_varpar.sh

# Testfunktionen generieren
# cd $repo_pfad/robot_codegen_scripts/
# source $repo_pfad/robot_codegen_scripts/testfunctions_generate.sh

