#!/bin/bash -e
# Hilfsskripte für die Matlab-Code-Generierung
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-05
# (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

repo_pfad=$(pwd)/..
# Initialisiere Variablen
source $repo_pfad/robot_codegen_definitions/robot_env_par.sh

# Umrechnung von Dreiecksmatrix einer symmetrischer Matrix als Vektor geschrieben in eine symmetrische Matrix
$repo_pfad/scripts/run_maple_script.sh $repo_pfad/helper/robot_gen_symmat2vector_parrob.mpl
# Umbennung der Ergebnisvariable ("unknown" bei Code-Generierung "1").
# Die Ergebnisvariable muss hier anders heißen, damit derselbe Name nicht zweimal vorkommt.
for f in `find "$repo_pfad/codeexport/$robot_name/tmp/" -name "vec2symmat*matlab.m"`; do
  sed -i "s/unknown/resmat/g" $f
done
