#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Dynamik (explizit) IC, fixed-base"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
template_pfad=$repo_pfad/robot_codegen_scripts/templates_sym
# Initialisiere Variablen
source robot_codegen_tmpvar_bash_ic.sh
source $repo_pfad/robot_codegen_definitions/robot_env_IC.sh

# Schleife für beide Basis-Darstellungen durchgehen.
# Die Methode "twist" muss am Ende kommen, damit Basis-unabhängige Funktionen mit den Ergebnissen dieser Methode
# generiert werden ("twist"-Code ist kürzer; Zuerst erstellte Datei mit "eulxyz" wird dann überschrieben).
basemethodenames=( eulxyz twist )

# Erstelle Matlab-Funktionen der explizit ausgerechneten Dynamik (nicht in Regressorform)
for (( dynpar=1; dynpar<=2; dynpar++ ))
do
  for basemeth in "${basemethodenames[@]}"
  do

    # Inverse Dynamik (Fixed Base)
    quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_fixb_par${dynpar}_matlab.m
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_invdynJ_fixb_slag_vp${dynpar}.m
    cat $head_pfad/robot_matlabtmp_invdynJ_fixb_par${dynpar}.head.m > $zieldat
    printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
    source robot_codegen_matlabfcn_postprocess_ic.sh $zieldat 0
    source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_qJDD.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
    cat $tmp_pfad/robot_matlabtmp_assert_m.m >> $zieldat
    if [ $dynpar == 1 ]; then
      cat $tmp_pfad/robot_matlabtmp_assert_rcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_Ic.m >> $zieldat
    else
      cat $tmp_pfad/robot_matlabtmp_assert_mrcom.m >> $zieldat
      cat $tmp_pfad/robot_matlabtmp_assert_If.m >> $zieldat
    fi
    cat ${template_pfad}/robot_IC_matlabtmp_invdynJ_fixb_par${dynpar}.m.template >> $zieldat
    
    source robot_codegen_matlabfcn_postprocess_ic.sh $zieldat 1 0 ${quelldat}.subsvar
  done # floatb_twist/floatb_eulxyz
done # par1/par2
