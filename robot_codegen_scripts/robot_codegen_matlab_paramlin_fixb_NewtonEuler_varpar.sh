#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Dynamik-Regressor nach Newton-Euler (Fixed Base)"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Erstelle Matlab-Funktionen der Regressorform
forcemaple=( tauJ tauB f m )
forcematlab=( invdynJ invdynB invdynf invdynm )
stringmaple=( regressor_minpar regressor )
stringmatlab=( regmin reg2 )

# Generiere zwei verschiedene Regressorformen (Bisher nur rm=2 implementiert):
# (Minimalparameterregressor (rm=1)) und Inertialparameterregressor (rm=2)
for (( rm=1; rm<=1; rm++ ))
do
	maple_string=${stringmaple[$rm]}
	matlab_string=${stringmatlab[$rm]}
	
	# Generiere Dynamik für: tauJ, tauB, Schnittkräfte f und Schnittmomente m
	for (( force=0; force<=3; force++ )) 
	do	
		maple_force=${forcemaple[$force]}
		matlab_force=${forcematlab[$force]}

		# Inverse Dynamik
		quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_fixb_NewtonEuler_${maple_force}_${maple_string}_matlab.m
		zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_${matlab_force}_fixb_NewtonEuler_${matlab_string}_slag_vp.m
		if [ -f $quelldat ]; then
			cat $head_pfad/robot_matlabtmp_${matlab_force}_fixb_NewtonEuler_${matlab_string}.head.m > $zieldat
			printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
			source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
			source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
			cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
			cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
			cat $tmp_pfad/robot_matlabtmp_assert_qJDD.m >> $zieldat
			cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
			cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
			printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
			cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
			cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
			cat $tmp_pfad/robot_matlabtmp_qJDD.m >> ${quelldat}.subsvar
			cat $tmp_pfad/robot_matlabtmp_g.m >> ${quelldat}.subsvar
			cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar
			printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
			sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
			cat $quelldat >> $zieldat
			source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
		else
			echo "Code in ${quelldat##*/} nicht gefunden."
		fi
	done
done
