#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Tim-David Job (Hiwi bei Moritz Schappler), 2018-04
# Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-04
# (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Dynamik nach Newton-Euler (explizit), fixed-base"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
template_pfad=$repo_pfad/robot_codegen_scripts/templates_sym
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

forcemaple=( tauJ tauB tauJB f_i_i m_i_i )
forcematlab=( invdynJ invdynB invdynJB invdynf invdynm )
basemethodenames=( twist ) # eulxyz

# Erstelle Matlab-Funktionen der explizit ausgerechneten Dynamik mit Newton-Euler (nicht in Regressorform)
for (( dynpar=2; dynpar<=2; dynpar++ )); do
  for basemeth in "${basemethodenames[@]}"; do
    # Generiere Dynamik für: tauJ, tauB, tauJB, Schnittkräfte f und Schnittmomente m
    for (( force=0; force<=4; force++ )); do
      maple_force=${forcemaple[$force]}
      matlab_force=${forcematlab[$force]}
      # Inverse Dynamik
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_fixb_NewtonEuler_linkframe_${maple_force}_par${dynpar}_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_${matlab_force}_fixb_snew_vp${dynpar}.m
      if [ -f $quelldat ]; then
        cat $head_pfad/robot_matlabtmp_${matlab_force}_fixb_NewtonEuler_par${dynpar}.head.m > $zieldat
        printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
        source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
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
        printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_qJDD.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
        cat $tmp_pfad/robot_matlabtmp_par_m.m >> ${quelldat}.subsvar
        if [ $dynpar == 1 ]; then
          cat $tmp_pfad/robot_matlabtmp_par_rcom.m >> ${quelldat}.subsvar
          cat $tmp_pfad/robot_matlabtmp_par_Ic.m >> ${quelldat}.subsvar
        else
          cat $tmp_pfad/robot_matlabtmp_par_mrcom.m >> ${quelldat}.subsvar
          cat $tmp_pfad/robot_matlabtmp_par_If.m >> ${quelldat}.subsvar
        fi
        printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
        sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
        cat $quelldat >> $zieldat
      else
        echo "Code in ${quelldat##*/} nicht gefunden."
      fi
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
    done # tauJ, tauB, tauJB, Schnittkräfte f und Schnittmomente m
  done # floatb_twist/floatb_eulangrpy
done # par1/par2
