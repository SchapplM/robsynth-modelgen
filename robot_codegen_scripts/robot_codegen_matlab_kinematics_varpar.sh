#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Kinematik"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/robot_codegen_scripts/tmp
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh

# Direkte Kinematik (Ohne Basis-Orientierung oder Höhe)
quelldat=$repo_pfad/codeexport/${robot_name}/tmp/fkine_mdh_floatb_twist_rotmat_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_fkine_fixb_rotmat_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat ${tmp_pfad}_head/robot_matlabtmp_fkine_fixb_rotmat.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
  echo "%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> $zieldat
  printf "rxs_base=0;\nrys_base=0;\nrzs_base=0;\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
  
  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
  varname_tmp=`grep "=" $zieldat| tail -1 | sed 's/\(.*\)=.*/\1/'`
  echo "T_ges = $varname_tmp;" >> $zieldat
  echo "%% Postprocessing: Reshape Output" >> $zieldat
  printf "%% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)\n" >> $zieldat
  printf "T_c_mdh = NaN(4,4,%%NJ%%+1);\nfor i = 1:%%NJ%%+1\n  T_c_mdh(:,:,i) = T_ges((i-1)*4+1 : 4*i, :);\nend\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Direkte Kinematik (Basis-Orientierung (EulerXYZ) und Position)
quelldat=$repo_pfad/codeexport/${robot_name}/tmp/fkine_mdh_floatb_eulangrpy_rotmat_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_fkine_floatb_eulangrpy_rotmat_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat ${tmp_pfad}_head/robot_matlabtmp_fkine_floatb_eulangrpy_rotmat.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_rB.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_rB.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_phiB.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
  
  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
  varname_tmp=`grep "=" $zieldat| tail -1 | sed 's/\(.*\)=.*/\1/'`
  echo "T_ges = $varname_tmp;" >> $zieldat
  echo "%% Postprocessing: Reshape Output" >> $zieldat
  printf "%% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)\n" >> $zieldat
  printf "T_c_mdh = NaN(4,4,%%NL%%);\nfor i = 1:%%NL%%\n  T_c_mdh(:,:,i) = T_ges((i-1)*4+1 : 4*i, :);\nend\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Gelenk-Transformationsmatrizen
quelldat=$repo_pfad/codeexport/${robot_name}/tmp/joint_transformation_mdh_rotmat_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_joint_trafo_rotmat_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat ${tmp_pfad}_head/robot_matlabtmp_joint_transformation.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
  
  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
  varname_tmp=`grep "=" $zieldat| tail -1 | sed 's/\(.*\)=.*/\1/'`
  echo "T_ges = $varname_tmp;" >> $zieldat
  echo "%% Postprocessing: Reshape Output" >> $zieldat
  printf "%% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)\n" >> $zieldat
  printf "T_mdh = NaN(4,4,%%NJ%%);\nfor i = 1:%%NJ%%\n  T_mdh(:,:,i) = T_ges((i-1)*4+1 : 4*i, :);\nend\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Jacobi-Matrizen für jeden Körper
# Reihenfolge:
# (1) analytisch, transl.
# (2) analytisch, rot.
# (3) geometrisch, rot.
# (4) analytisch, transl., Zeitableitung
# (5) analytisch, rot., Zeitableitung
# (6) geometrisch, rot., Zeitableitung
# (7) geometrisch (vollständig)
# (8) analytisch (vollständig)
# (9) geometrisch, Zeitableitung (vollständig)
# (10) analytisch, Zeitableitung (vollständig)

for (( jacart=1; jacart<=10; jacart++ ))
do
  for (( ib=1; ib<=$robot_NL; ib++ ))
  do
    if [ "$jacart" -eq "1" ]; then
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobia_transl_${ib}_floatb_twist_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobia_transl_${ib}_floatb_twist_sym_varpar.m
      headdat=${tmp_pfad}_head/robot_matlabtmp_jacobia_transl.head.m
    elif [ "$jacart" -eq "2" ]; then
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobia_rot_${ib}_floatb_twist_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobia_rot_${ib}_floatb_twist_sym_varpar.m
      headdat=${tmp_pfad}_head/robot_matlabtmp_jacobia_rot.head.m
    elif [ "$jacart" -eq "3" ]; then
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobig_rot_${ib}_floatb_twist_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobig_rot_${ib}_floatb_twist_sym_varpar.m
      headdat=${tmp_pfad}_head/robot_matlabtmp_jacobig_rot.head.m
    elif [ "$jacart" -eq "4" ]; then
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobiaD_transl_${ib}_floatb_twist_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiaD_transl_${ib}_floatb_twist_sym_varpar.m
      headdat=${tmp_pfad}_head/robot_matlabtmp_jacobiaD_transl.head.m
    elif [ "$jacart" -eq "5" ]; then
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobiaD_rot_${ib}_floatb_twist_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiaD_rot_${ib}_floatb_twist_sym_varpar.m
      headdat=${tmp_pfad}_head/robot_matlabtmp_jacobiaD_rot.head.m
    elif [ "$jacart" -eq "6" ]; then
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobigD_rot_${ib}_floatb_twist_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobigD_rot_${ib}_floatb_twist_sym_varpar.m
      headdat=${tmp_pfad}_head/robot_matlabtmp_jacobigD_rot.head.m
    elif [ "$jacart" -eq "7" ]; then
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobig_${ib}_floatb_twist_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobig_${ib}_floatb_twist_sym_varpar.m
      headdat=${tmp_pfad}_head/robot_matlabtmp_jacobig.head.m
    elif [ "$jacart" -eq "8" ]; then
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobia_${ib}_floatb_twist_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobia_${ib}_floatb_twist_sym_varpar.m
      headdat=${tmp_pfad}_head/robot_matlabtmp_jacobia.head.m
    elif [ "$jacart" -eq "9" ]; then
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobigD_${ib}_floatb_twist_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobigD_${ib}_floatb_twist_sym_varpar.m
      headdat=${tmp_pfad}_head/robot_matlabtmp_jacobigD.head.m
    elif [ "$jacart" -eq "10" ]; then
      quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobiaD_${ib}_floatb_twist_matlab.m
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiaD_${ib}_floatb_twist_sym_varpar.m
      headdat=${tmp_pfad}_head/robot_matlabtmp_jacobiaD.head.m
    fi;
    if [ "$jacart" -eq "4" ] || [ "$jacart" -eq "5" ] || [ "$jacart" -eq "6" ] || [ "$jacart" -gt "8" ]; then # Zeitableitung, Geschwindigkeit ist Eingang
      input_qD=true
    else
      input_qD=false
    fi;
    if [ "$jacart" -eq "1" ] || [ "$jacart" -eq "4" ] || [ "$jacart" -gt "6" ]; then # translatorisch oder Gesamtmatrix
      input_r=true
    else
      input_r=false
    fi;

    if [ "$jacart" -lt "7" ]; then # Einfügen von Quelltext
      if [ -f $quelldat ]; then
        cat $headdat > $zieldat
        printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
        if [ "$input_qD" == "true" ]; then
          cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
        fi
        if [ "$input_r" == "true" ]; then
          printf "assert(isa(r_i_i_C,'double') && isreal(r_i_i_C) && all(size(r_i_i_C) == [3 1]), ..." >> $zieldat
          printf "\n\t'%%FN%%: Position vector r_i_i_C has to be [3x1] double');\n" >> $zieldat
        fi
        cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
      
        echo "%% Variable Initialization" >> $zieldat
        cat $tmp_pfad/robot_matlabtmp_qJ.m >> $zieldat
        if [ "$input_qD" == "true" ]; then
          cat $tmp_pfad/robot_matlabtmp_qJD.m >> $zieldat
        fi;
        if [ "$input_r" == "true" ]; then
          printf "\npx = r_i_i_C(1);\npy = r_i_i_C(2);\npz = r_i_i_C(3);\n" >> $zieldat
        fi;
        cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
        
        printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
        sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
        cat $quelldat >> $zieldat
      else
        echo "Code in ${quelldat##*/} nicht gefunden."
        continue
      fi;
    else
      cat $headdat > $zieldat # reines Aufrufen anderer Funktionen
    fi;
    if [ "$jacart" -lt "7" ]; then
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 1
    else
      source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
	fi;
    # Markierung ersetzen mit Segmentnummer der Jacobi-Matrix
    sed -i "s/%LIJAC%/${ib}/g" $zieldat
  done
done

# Jacobi-Matrix für alle Körper zusammen
# (1) geometrisch (vollständig)
# (2) analytisch (vollständig)
# (3) geometrisch, rot.
# (4) analytisch, rot.
# (5) geometrisch, Zeitableitung (vollständig)
# (6) analytisch, Zeitableitung (vollständig)
# (7) geometrisch, Zeitableitung, rot.
# (8) analytisch, Zeitableitung, rot.

for (( jacart=1; jacart<=8; jacart++ ))
do
  if [ "$jacart" -eq "1" ]; then
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobig_floatb_twist_sym_varpar.m
    headdat=${tmp_pfad}_head/robot_matlabtmp_jacobig_all.head.m
  elif [ "$jacart" -eq "2" ]; then
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobia_floatb_twist_sym_varpar.m
    headdat=${tmp_pfad}_head/robot_matlabtmp_jacobia_all.head.m
  elif [ "$jacart" -eq "3" ]; then
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobig_rot_floatb_twist_sym_varpar.m
    headdat=${tmp_pfad}_head/robot_matlabtmp_jacobig_rot_all.head.m
  elif [ "$jacart" -eq "4" ]; then
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobia_rot_floatb_twist_sym_varpar.m
    headdat=${tmp_pfad}_head/robot_matlabtmp_jacobia_rot_all.head.m
  elif [ "$jacart" -eq "5" ]; then
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobigD_floatb_twist_sym_varpar.m
    headdat=${tmp_pfad}_head/robot_matlabtmp_jacobigD_all.head.m
  elif [ "$jacart" -eq "6" ]; then
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiaD_floatb_twist_sym_varpar.m
    headdat=${tmp_pfad}_head/robot_matlabtmp_jacobiaD_all.head.m
  elif [ "$jacart" -eq "7" ]; then
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobigD_rot_floatb_twist_sym_varpar.m
    headdat=${tmp_pfad}_head/robot_matlabtmp_jacobigD_rot_all.head.m
  elif [ "$jacart" -eq "8" ]; then
    zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiaD_rot_floatb_twist_sym_varpar.m
    headdat=${tmp_pfad}_head/robot_matlabtmp_jacobiaD_rot_all.head.m
  fi;
  if [ "$jacart" -gt "4" ]; then # Zeitableitung: Geschwindigkeit als Eingang
    input_qD=true
  else
    input_qD=false
  fi;
  if [ "$jacart" -eq "1" ] || [ "$jacart" -eq "2" ] || [ "$jacart" -eq "5" ] || [ "$jacart" -eq "6" ]; then # Vollständige Matrix: Position als Eingang
    input_r=true
  else
    input_r=false
  fi;
  cat $headdat > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  if [ "$input_qD" == "true" ]; then
    cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
  fi;
  if [ "$input_r" == "true" ]; then
    printf "assert(isa(r_i_i_C,'double') && isreal(r_i_i_C) && all(size(r_i_i_C) == [3 1]), ..." >> $zieldat
    printf "\n\t'%%FN%%: Position vector r_i_i_C has to be [3x1] double');\n" >> $zieldat
  fi;
  printf "assert(isa(link_index,'uint8') && all(size(link_index) == [1 1]), ..." >> $zieldat
  printf "\n\t'%%FN%%: link_index has to be [1x1] uint8');\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Function calls" >> $zieldat
  for (( ib=1; ib<=$robot_NL; ib++ ))
  do
    if [ "$ib" -eq "1" ]; then
      printf "if link_index == ${ib}\n" >> $zieldat
    else
      printf "elseif link_index == ${ib}\n" >> $zieldat
    fi;
    if [ "$jacart" -eq "1" ]; then
      printf "\tJg=${robot_name}_jacobig_${ib}_floatb_twist_sym_varpar(qJ, r_i_i_C, pkin);\n" >> $zieldat
    elif [ "$jacart" -eq "2" ]; then
      printf "\tJa=${robot_name}_jacobia_${ib}_floatb_twist_sym_varpar(qJ, r_i_i_C, pkin);\n" >> $zieldat
    elif [ "$jacart" -eq "3" ]; then
      printf "\tJg_rot=${robot_name}_jacobig_rot_${ib}_floatb_twist_sym_varpar(qJ, pkin);\n" >> $zieldat
    elif [ "$jacart" -eq "4" ]; then
      printf "\tJa_rot=${robot_name}_jacobia_rot_${ib}_floatb_twist_sym_varpar(qJ, pkin);\n" >> $zieldat
    elif [ "$jacart" -eq "5" ]; then
      printf "\tJgD=${robot_name}_jacobigD_${ib}_floatb_twist_sym_varpar(qJ, qJD, r_i_i_C, pkin);\n" >> $zieldat
    elif [ "$jacart" -eq "6" ]; then
      printf "\tJaD=${robot_name}_jacobiaD_${ib}_floatb_twist_sym_varpar(qJ, qJD, r_i_i_C, pkin);\n" >> $zieldat
    elif [ "$jacart" -eq "7" ]; then
      printf "\tJgD_rot=${robot_name}_jacobigD_rot_${ib}_floatb_twist_sym_varpar(qJ, qJD, pkin);\n" >> $zieldat
    elif [ "$jacart" -eq "8" ]; then
      printf "\tJaD_rot=${robot_name}_jacobiaD_rot_${ib}_floatb_twist_sym_varpar(qJ, qJD, pkin);\n" >> $zieldat
    fi;
  done
  if [ "$jacart" -eq "1" ]; then
    printf "else\n\tJg=NaN(6,$robot_NQJ);\nend" >> $zieldat
  elif [ "$jacart" -eq "2" ]; then
    printf "else\n\tJa=NaN(6,$robot_NQJ);\nend" >> $zieldat
  elif [ "$jacart" -eq "3" ]; then
    printf "else\n\tJg_rot=NaN(3,$robot_NQJ);\nend" >> $zieldat
  elif [ "$jacart" -eq "4" ]; then
    printf "else\n\tJa_rot=NaN(3,$robot_NQJ);\nend" >> $zieldat
  elif [ "$jacart" -eq "5" ]; then
    printf "else\n\tJgD=NaN(6,$robot_NQJ);\nend" >> $zieldat
  elif [ "$jacart" -eq "6" ]; then
    printf "else\n\tJaD=NaN(6,$robot_NQJ);\nend" >> $zieldat
  elif [ "$jacart" -eq "7" ]; then
    printf "else\n\tJgD_rot=NaN(6,$robot_NQJ);\nend" >> $zieldat
  elif [ "$jacart" -eq "8" ]; then
    printf "else\n\tJaD_rot=NaN(6,$robot_NQJ);\nend" >> $zieldat
  fi;
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
done

# Funktionen für explizite kinematische Zwangsbedingungen
quelldat=$repo_pfad/codeexport/${robot_name}/tmp/kinconstr_expl_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_kinconstr_expl_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat ${tmp_pfad}_head/robot_matlabtmp_kinconstr_expl.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
  
  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 1
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

quelldat=$repo_pfad/codeexport/${robot_name}/tmp/kinconstr_expl_jacobian_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_kinconstr_expl_jacobian_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat ${tmp_pfad}_head/robot_matlabtmp_kinconstr_expl_jacobian.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
  
  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

quelldat=$repo_pfad/codeexport/${robot_name}/tmp/kinconstr_expl_jacobianD_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_kinconstr_expl_jacobianD_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat ${tmp_pfad}_head/robot_matlabtmp_kinconstr_expl_jacobianD.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Variable Initialization" >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_qJD.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> $zieldat
  
  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi