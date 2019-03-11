#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur parameterlinearen Form der Floating-Base-Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2017-02
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Dynamik-Regressor nach Newton-Euler (Floating Base)"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

basemethodenames=( eulxyz ) # twist
forcemaple=( tauJ tauB f m )
forcematlab=( invdynJ invdynB invdynf invdynm )
stringmaple=( regressor_minpar regressor )
stringmatlab=( regmin reg2 )

# Erstelle Matlab-Funktionen der Regressorform für Floating Base Modell
for basemeth in "${basemethodenames[@]}"
do
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
			quelldat=$repo_pfad/codeexport/${robot_name}/tmp/invdyn_floatb_${basemeth}_NewtonEuler_${maple_force}_${matlab_string}_matlab.m
			zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_${matlab_force}_floatb_${basemeth}_${matlab_string}_snew_vp.m
			if [ -f $quelldat ]; then
				cat $head_pfad/robot_matlabtmp_${matlab_force}_floatb_${basemeth}_NewtonEuler_${matlab_string}.head.m > $zieldat
				printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
				source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
				source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
				cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
				cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
				cat $tmp_pfad/robot_matlabtmp_assert_qJDD.m >> $zieldat
				if [ $basemeth == "twist" ]; then
					cat $tmp_pfad/robot_matlabtmp_assert_vB.m >> $zieldat
					cat $tmp_pfad/robot_matlabtmp_assert_aB.m >> $zieldat
				else
					cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
					cat $tmp_pfad/robot_matlabtmp_assert_xDB.m >> $zieldat
					cat $tmp_pfad/robot_matlabtmp_assert_xDDB.m >> $zieldat
				fi
				cat $tmp_pfad/robot_matlabtmp_assert_g.m >> $zieldat
				cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
				printf "\n%%%% Variable Initialization" > ${quelldat}.subsvar
				cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
				cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
				cat $tmp_pfad/robot_matlabtmp_qJDD.m >> ${quelldat}.subsvar
				if [ $basemeth == "twist" ]; then
					cat $tmp_pfad/robot_matlabtmp_vB.m >> ${quelldat}.subsvar
					cat $tmp_pfad/robot_matlabtmp_aB.m >> ${quelldat}.subsvar
				else
					cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
					cat $tmp_pfad/robot_matlabtmp_xDB.m >> ${quelldat}.subsvar
					cat $tmp_pfad/robot_matlabtmp_xDDB.m >> ${quelldat}.subsvar
				fi
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
done
