#!/bin/bash
# Passe die Vorlagen für Dynamik-Testfunktionen an den Roboter an.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (c) Institut für Regelungstechnik, Leibniz Universität Hannover

repo_pfad=$(pwd)/..
testfcn_pfad=$repo_pfad/robot_codegen_testfunctions
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

cp $testfcn_pfad/robot_varpar_invdyn_test.m.template $testfcn_pfad/robot_varpar_invdyn_test.m
source robot_codegen_matlabfcn_postprocess.sh $testfcn_pfad/robot_varpar_invdyn_test.m 0

cp $testfcn_pfad/robot_varpar_floatbase_test.m.template $testfcn_pfad/robot_varpar_floatbase_test.m
source robot_codegen_matlabfcn_postprocess.sh $testfcn_pfad/robot_varpar_floatbase_test.m 0

cp $testfcn_pfad/robot_varpar_testfunctions_parameter.m.template $testfcn_pfad/robot_varpar_testfunctions_parameter.m
source robot_codegen_matlabfcn_postprocess.sh $testfcn_pfad/robot_varpar_testfunctions_parameter.m 0
