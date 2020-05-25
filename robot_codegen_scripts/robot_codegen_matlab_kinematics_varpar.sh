#!/bin/bash -e
# Erstelle fertige Matlab-Funktionen aus exportiertem Code von Maple
# Dieses Skript erstellt die Funktionen zur Dynamik und wird von robot_codegen_matlab_varpar.sh aufgerufen.
#
# Dieses Skript im Ordner ausführen, in dem es im Repo liegt

# Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
# (C) Institut für Regelungstechnik, Leibniz Universität Hannover

echo "Generiere Matlabfunktionen: Kinematik"

repo_pfad=$(pwd)/..
tmp_pfad=$repo_pfad/workdir/tmp
head_pfad=$repo_pfad/robot_codegen_scripts/tmp_head
# Initialisiere Variablen
source robot_codegen_tmpvar_bash.sh
source $repo_pfad/robot_codegen_definitions/robot_env.sh
# Auswahl, ob Jacobi-Matrix-Funktionen für jedes Segment einzeln erstellt werden soll
gen_single_jacobi_files=0

# Direkte Kinematik (Ohne Basis-Orientierung oder Höhe)
quelldat=$repo_pfad/codeexport/${robot_name}/tmp/fkine_mdh_floatb_twist_rotmat_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_fkine_fixb_rotmat_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_fkine_fixb_rotmat.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
  source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat
  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
  printf "rxs_base = 0;\nrys_base = 0;\nrzs_base = 0;\n" >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
  varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
  echo "Tc_stack = $varname_tmp;" >> $zieldat
  echo "%% Postprocessing: Reshape Output" >> $zieldat
  printf "%% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)\n" >> $zieldat
  printf "%% Fallunterscheidung der Initialisierung für symbolische Eingabe\n" >> $zieldat
  printf "if isa([qJ; pkin], 'double'), Tc_mdh = NaN(4,4,%%NJ%%+1);               %% numerisch\n" >> $zieldat
  printf "else,                         Tc_mdh = sym('xx', [4,4,%%NJ%%+1]); end %% symbolisch\n" >> $zieldat
  printf "for i = 1:%%NJ%%+1\n  Tc_mdh(:,:,i) = [Tc_stack((i-1)*3+1 : 3*i, :);[0 0 0 1]];\nend\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0 0 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Direkte Kinematik (Basis-Orientierung (EulerXYZ) und Position)
quelldat=$repo_pfad/codeexport/${robot_name}/tmp/fkine_mdh_floatb_eulxyz_rotmat_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_fkine_floatb_eulxyz_rotmat_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_fkine_floatb_eulxyz_rotmat.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
  source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_rB.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_phiB.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_rB.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_phiB.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
  varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
  echo "Tc_stack = $varname_tmp;" >> $zieldat
  echo "%% Postprocessing: Reshape Output" >> $zieldat
  printf "%% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)\n" >> $zieldat
  printf "Tc_mdh = NaN(4,4,%%NL%%);\nfor i = 1:%%NL%%\n  Tc_mdh(:,:,i) = [Tc_stack((i-1)*4+1 : 4*i, :); [0 0 0 1]];\nend\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0 0 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Gelenk-Transformationsmatrizen
quelldat=$repo_pfad/codeexport/${robot_name}/tmp/joint_transformation_mdh_rotmat_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_joint_trafo_rotmat_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_joint_transformation.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
  source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  # Benenne die Ergebnisvariable des exportierten Codes um (zusätzlich zu Hilfsskript robot_codegen_matlabfcn_postprocess.sh)
  varname_tmp=`$repo_pfad/scripts/get_last_variable_name.sh $zieldat`
  echo "T_stack = $varname_tmp;" >> $zieldat
  echo "%% Postprocessing: Reshape Output" >> $zieldat
  printf "%% Convert Maple format (2-dimensional tensor) to Matlab format (3-dimensional tensor)\n" >> $zieldat

  printf "%% Fallunterscheidung der Initialisierung für symbolische Eingabe\n" >> $zieldat
  printf "if isa([qJ; pkin], 'double'), T_mdh = NaN(4,4,%%NJ%%);             %% numerisch\n" >> $zieldat
  printf "else,                         T_mdh = sym('xx', [4,4,%%NJ%%]); end %% symbolisch\n" >> $zieldat
  printf "\nfor i = 1:%%NJ%%\n  T_mdh(:,:,i) = [T_stack((i-1)*3+1 : 3*i, :);[0 0 0 1]];\nend\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0 0 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

# Jacobi-Matrix-Funktionen für jedes Segment einzeln erstellen
if [ "$gen_single_jacobi_files" -eq "1" ]; then
  # Jacobi-Matrizen für jeden Körper einzeln
  # Reihenfolge:
  # (1) a_transl: analytisch, transl. (nur für einen Körper)
  # (2) a_rot: analytisch, rot.
  # (3) g_rot: geometrisch, rot.
  # (4) aD_transl: analytisch, transl., Zeitableitung
  # (5) aD_rot: analytisch, rot., Zeitableitung
  # (6) gD_rot: geometrisch, rot., Zeitableitung
  # (7) R_rot: Ableitung der Rotationsmatrix (nur Rotationsteil)
  # (8) RD_rot: Ableitung der Rotationsmatrix, Zeitableitung (nur Rotationsteil)
  # (9) g: geometrisch (vollständig)
  # (10) a: analytisch (vollständig)
  # (11) gD: geometrisch, Zeitableitung (vollständig)
  # (12) aD: analytisch, Zeitableitung (vollständig)

  for (( jacart=1; jacart<=12; jacart++ ))
  do
    for (( ib=0; ib<$robot_NL; ib++ ))
    do
      if [ "$jacart" -eq "1" ]; then
        quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobia_transl_${ib}_floatb_twist_matlab.m
        zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobia_transl_${ib}_sym_varpar.m
        headdat=$head_pfad/robot_matlabtmp_jacobia_transl.head.m
      elif [ "$jacart" -eq "2" ]; then
        quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobia_rot_${ib}_floatb_twist_matlab.m
        zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobia_rot_${ib}_sym_varpar.m
        headdat=$head_pfad/robot_matlabtmp_jacobia_rot.head.m
      elif [ "$jacart" -eq "3" ]; then
        quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobig_rot_${ib}_floatb_twist_matlab.m
        zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobig_rot_${ib}_sym_varpar.m
        headdat=$head_pfad/robot_matlabtmp_jacobig_rot.head.m
      elif [ "$jacart" -eq "4" ]; then
        quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobiaD_transl_${ib}_floatb_twist_matlab.m
        zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiaD_transl_${ib}_sym_varpar.m
        headdat=$head_pfad/robot_matlabtmp_jacobiaD_transl.head.m
      elif [ "$jacart" -eq "5" ]; then
        quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobiaD_rot_${ib}_floatb_twist_matlab.m
        zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiaD_rot_${ib}_sym_varpar.m
        headdat=$head_pfad/robot_matlabtmp_jacobiaD_rot.head.m
      elif [ "$jacart" -eq "6" ]; then
        quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobigD_rot_${ib}_floatb_twist_matlab.m
        zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobigD_rot_${ib}_sym_varpar.m
        headdat=$head_pfad/robot_matlabtmp_jacobigD_rot.head.m
      elif [ "$jacart" -eq "7" ]; then
        quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobiR_rot_${ib}_floatb_twist_matlab.m
        zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiR_rot_${ib}_sym_varpar.m
        headdat=$head_pfad/robot_matlabtmp_jacobiR_rot.head.m
      elif [ "$jacart" -eq "8" ]; then
        quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobiRD_rot_${ib}_floatb_twist_matlab.m
        zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiRD_rot_${ib}_sym_varpar.m
        headdat=$head_pfad/robot_matlabtmp_jacobiRD_rot.head.m
      elif [ "$jacart" -eq "9" ]; then
        quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobig_${ib}_floatb_twist_matlab.m
        zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobig_${ib}_sym_varpar.m
        headdat=$head_pfad/robot_matlabtmp_jacobig.head.m
      elif [ "$jacart" -eq "10" ]; then
        quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobia_${ib}_floatb_twist_matlab.m
        zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobia_${ib}_sym_varpar.m
        headdat=$head_pfad/robot_matlabtmp_jacobia.head.m
      elif [ "$jacart" -eq "11" ]; then
        quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobigD_${ib}_floatb_twist_matlab.m
        zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobigD_${ib}_sym_varpar.m
        headdat=$head_pfad/robot_matlabtmp_jacobigD.head.m
      elif [ "$jacart" -eq "12" ]; then
        quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobiaD_${ib}_floatb_twist_matlab.m
        zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiaD_${ib}_sym_varpar.m
        headdat=$head_pfad/robot_matlabtmp_jacobiaD.head.m
      fi;
      if [ "$jacart" -eq "4" ] || [ "$jacart" -eq "5" ] || [ "$jacart" -eq "6" ] || [ "$jacart" -eq "8" ] || [ "$jacart" -gt "10" ]; then # Zeitableitung, Geschwindigkeit ist Eingang
        input_qD=true
      else
        input_qD=false
      fi;
      if [ "$jacart" -eq "1" ] || [ "$jacart" -eq "4" ] || [ "$jacart" -gt "8" ]; then # translatorisch oder Gesamtmatrix
        input_r=true
      else
        input_r=false
      fi;

      if [ "$jacart" -lt "9" ]; then # Einfügen von Quelltext
        if [ -f $quelldat ]; then
          cat $headdat > $zieldat
          printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
          source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
          source $repo_pfad/scripts/set_inputdim_line.sh $zieldat "r_i_i_C|zeros(3,1)"
          cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
          if [ "$input_qD" == "true" ]; then
            cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
          fi
          if [ "$input_r" == "true" ]; then
            printf "assert(isa(r_i_i_C,'double') && isreal(r_i_i_C) && all(size(r_i_i_C) == [3 1]), ..." >> $zieldat
            printf "\n\t'%%FN%%: Position vector r_i_i_C has to be [3x1] double');\n" >> $zieldat
          fi
          cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

          echo "%% Variable Initialization" > ${quelldat}.subsvar
          cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
          if [ "$input_qD" == "true" ]; then
            cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
          fi;
          if [ "$input_r" == "true" ]; then
            printf "\npx = r_i_i_C(1);\npy = r_i_i_C(2);\npz = r_i_i_C(3);\n" >> ${quelldat}.subsvar
          fi;
          cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

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
      if [ "$jacart" -lt "9" ]; then
        source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
      else
        source robot_codegen_matlabfcn_postprocess.sh $zieldat 0 0 ${quelldat}.subsvar
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
  # (5) Ableitung der Rotationsmatrix (nur Rotation)
  # (6) analytisch, translatorischer Teil
  # (7) geometrisch, Zeitableitung (vollständig)
  # (8) analytisch, Zeitableitung (vollständig)
  # (9) geometrisch, Zeitableitung, rot.
  # (10) analytisch, Zeitableitung, rot.
  # (11) analytisch, Zeitableitung, translatorisch.
  # (12) Ableitung der Rotationsmatrix, Zeitableitung (nur Rotation)
  
  for (( jacart=1; jacart<=12; jacart++ ))
  do
    if [ "$jacart" -eq "1" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobig_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobig_all.head.m
    elif [ "$jacart" -eq "2" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobia_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobia_all.head.m
    elif [ "$jacart" -eq "3" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobig_rot_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobig_rot_all.head.m
    elif [ "$jacart" -eq "4" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobia_rot_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobia_rot_all.head.m
    elif [ "$jacart" -eq "5" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiR_rot_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobiR_rot_all.head.m
    elif [ "$jacart" -eq "6" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobia_transl_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobia_transl_all.head.m
    elif [ "$jacart" -eq "7" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobigD_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobigD_all.head.m
    elif [ "$jacart" -eq "8" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiaD_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobiaD_all.head.m
    elif [ "$jacart" -eq "9" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobigD_rot_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobigD_rot_all.head.m
    elif [ "$jacart" -eq "10" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiaD_rot_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobiaD_rot_all.head.m
    elif [ "$jacart" -eq "11" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiaD_transl_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobiaD_transl_all.head.m
    elif [ "$jacart" -eq "12" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiRD_rot_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobiRD_rot_all.head.m
    fi;
    if [ "$jacart" -gt "6" ]; then # Zeitableitung: Geschwindigkeit als Eingang
      input_qD=true
    else
      input_qD=false
    fi;
    if [ "$jacart" -eq "1" ] || [ "$jacart" -eq "2" ] || [ "$jacart" -eq "6" ] || [ "$jacart" -eq "7" ] || [ "$jacart" -eq "8" ] || [ "$jacart" -eq "11" ]; then # Vollständige Matrix: Position (des Bezugspunktes) als Eingang
      input_r=true
    else
      input_r=false
    fi;
    cat $headdat > $zieldat
    printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
    source $repo_pfad/scripts/set_inputdim_line.sh $zieldat "r_i_i_C|zeros(3,1);link_index|uint8(0)"
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
    for (( ib=0; ib<$robot_NL; ib++ ))
    do
      if [ "$ib" -eq "0" ]; then
        printf "if link_index == ${ib}\n" >> $zieldat
      else
        printf "elseif link_index == ${ib}\n" >> $zieldat
      fi;
      if [ "$jacart" -eq "1" ]; then
        printf "\tJg=${robot_name}_jacobig_${ib}_sym_varpar(qJ, r_i_i_C, pkin);\n" >> $zieldat
      elif [ "$jacart" -eq "2" ]; then
        printf "\tJa=${robot_name}_jacobia_${ib}_sym_varpar(qJ, r_i_i_C, pkin);\n" >> $zieldat
      elif [ "$jacart" -eq "3" ]; then
        printf "\tJg_rot=${robot_name}_jacobig_rot_${ib}_sym_varpar(qJ, pkin);\n" >> $zieldat
      elif [ "$jacart" -eq "4" ]; then
        printf "\tJa_rot=${robot_name}_jacobia_rot_${ib}_sym_varpar(qJ, pkin);\n" >> $zieldat
      elif [ "$jacart" -eq "5" ]; then
        printf "\tJR_rot=${robot_name}_jacobiR_rot_${ib}_sym_varpar(qJ, pkin);\n" >> $zieldat
      elif [ "$jacart" -eq "6" ]; then
        printf "\tJa_transl=${robot_name}_jacobia_transl_${ib}_sym_varpar(qJ, r_i_i_C, pkin);\n" >> $zieldat
      elif [ "$jacart" -eq "7" ]; then
        printf "\tJgD=${robot_name}_jacobigD_${ib}_sym_varpar(qJ, qJD, r_i_i_C, pkin);\n" >> $zieldat
      elif [ "$jacart" -eq "8" ]; then
        printf "\tJaD=${robot_name}_jacobiaD_${ib}_sym_varpar(qJ, qJD, r_i_i_C, pkin);\n" >> $zieldat
      elif [ "$jacart" -eq "9" ]; then
        printf "\tJgD_rot=${robot_name}_jacobigD_rot_${ib}_sym_varpar(qJ, qJD, pkin);\n" >> $zieldat
      elif [ "$jacart" -eq "10" ]; then
        printf "\tJaD_rot=${robot_name}_jacobiaD_rot_${ib}_sym_varpar(qJ, qJD, pkin);\n" >> $zieldat
      elif [ "$jacart" -eq "11" ]; then
        printf "\tJaD_transl=${robot_name}_jacobiaD_transl_${ib}_sym_varpar(qJ, qJD, r_i_i_C, pkin);\n" >> $zieldat
       elif [ "$jacart" -eq "12" ]; then
        printf "\tJRD_rot=${robot_name}_jacobiRD_rot_${ib}_sym_varpar(qJ, qJD, pkin);\n" >> $zieldat
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
      printf "else\n\tJR_rot=NaN(9,$robot_NQJ);\nend" >> $zieldat
    elif [ "$jacart" -eq "6" ]; then
      printf "else\n\tJa_transl=NaN(3,$robot_NQJ);\nend" >> $zieldat
    elif [ "$jacart" -eq "7" ]; then
      printf "else\n\tJgD=NaN(6,$robot_NQJ);\nend" >> $zieldat
    elif [ "$jacart" -eq "8" ]; then
      printf "else\n\tJaD=NaN(6,$robot_NQJ);\nend" >> $zieldat
    elif [ "$jacart" -eq "9" ]; then
      printf "else\n\tJgD_rot=NaN(3,$robot_NQJ);\nend" >> $zieldat
    elif [ "$jacart" -eq "10" ]; then
      printf "else\n\tJaD_rot=NaN(3,$robot_NQJ);\nend" >> $zieldat
    elif [ "$jacart" -eq "11" ]; then
      printf "else\n\tJaD_transl=NaN(3,$robot_NQJ);\nend" >> $zieldat
    elif [ "$jacart" -eq "12" ]; then
      printf "else\n\tJRD_rot=NaN(9,$robot_NQJ);\nend" >> $zieldat
    fi;
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
  done

# Oder eine Datei für die Jacobi-Matrizen aller Segmente erstellen
else
  # (1) geometrisch, rot.
  # (2) analytisch, rot.
  # (3) Ableitung der Rotationsmatrix (nur Rotation)
  # (4) Ableitung der Rotationsmatrix, Zeitableitung (nur Rotation)
  # (5) analytisch, translatorischer Teil
  # (6) geometrisch, Zeitableitung, rot.
  # (7) analytisch, Zeitableitung, rot.
  # (8) analytisch, Zeitableitung, translatorisch.
  # (9) geometrisch (vollständig)
  # (10) analytisch (vollständig)
  # (11) geometrisch, Zeitableitung (vollständig)
  # (12) analytisch, Zeitableitung (vollständig)

  for (( jacart=1; jacart<=11; jacart++ ))
  do
    if [ "$jacart" -eq "1" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobig_rot_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobig_rot_all.head.m
    elif [ "$jacart" -eq "2" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobia_rot_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobia_rot_all.head.m
    elif [ "$jacart" -eq "3" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiR_rot_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobiR_rot_all.head.m
     elif [ "$jacart" -eq "4" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiRD_rot_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobiRD_rot_all.head.m
    elif [ "$jacart" -eq "5" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobia_transl_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobia_transl_all.head.m
    elif [ "$jacart" -eq "6" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobigD_rot_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobigD_rot_all.head.m
    elif [ "$jacart" -eq "7" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiaD_rot_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobiaD_rot_all.head.m
    elif [ "$jacart" -eq "8" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiaD_transl_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobiaD_transl_all.head.m
    elif [ "$jacart" -eq "9" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobig_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobig_all_2.head.m
    elif [ "$jacart" -eq "10" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobia_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobia_all_2.head.m
    elif [ "$jacart" -eq "11" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobigD_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobigD_all_2.head.m
    elif [ "$jacart" -eq "12" ]; then
      zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_jacobiaD_sym_varpar.m
      headdat=$head_pfad/robot_matlabtmp_jacobiaD_all_2.head.m
    fi;
    if [ "$jacart" -eq "4" ] || [ "$jacart" -eq "6" ] || [ "$jacart" -eq "7" ] || [ "$jacart" -eq "8" ] || [ "$jacart" -eq "11" ] || [ "$jacart" -eq "12" ]; then # Zeitableitung: Geschwindigkeit als Eingang
      input_qD=true
    else
      input_qD=false
    fi;
    
    if [ "$jacart" -eq "5" ] || [ "$jacart" -gt "7" ]; then # translatorisch oder Gesamtmatrix
      input_r=true
    else
      input_r=false
    fi;
    
    cat $headdat > $zieldat
    printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
    source $repo_pfad/scripts/set_inputdim_line.sh $zieldat "r_i_i_C|zeros(3,1);link_index|uint8(0)"
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
    # Definition der Ausgabevariable
    if [ "$jacart" -eq "1" ]; then
      printf "Jg_rot=NaN(3,$robot_NQJ);\n" >> $zieldat
    elif [ "$jacart" -eq "2" ]; then
      printf "Ja_rot=NaN(3,$robot_NQJ);\n" >> $zieldat
    elif [ "$jacart" -eq "3" ]; then
      printf "JR_rot=NaN(9,$robot_NQJ);\n" >> $zieldat
    elif [ "$jacart" -eq "4" ]; then
      printf "JRD_rot=NaN(9,$robot_NQJ);\n" >> $zieldat
    elif [ "$jacart" -eq "5" ]; then
      printf "Ja_transl=NaN(3,$robot_NQJ);\n" >> $zieldat
    elif [ "$jacart" -eq "6" ]; then
      printf "JgD_rot=NaN(3,$robot_NQJ);\n" >> $zieldat
    elif [ "$jacart" -eq "7" ]; then
      printf "JaD_rot=NaN(3,$robot_NQJ);\n" >> $zieldat
    elif [ "$jacart" -eq "8" ]; then
      printf "JaD_transl=NaN(3,$robot_NQJ);\n" >> $zieldat
    fi;
    
    # Fallunterscheidung für einzelne Körper
    if [ "$jacart" -lt "9" ]; then # Kopieren des symbolischen Codes der einzelnen Teilkörper für rot und transl
      for (( ib=0; ib<$robot_NL; ib++ ))
      do
        if [ "$ib" -eq "0" ]; then
          printf "if link_index == ${ib}\n" >> $zieldat
        else
          printf "elseif link_index == ${ib}\n" >> $zieldat
        fi;
        if [ "$jacart" -eq "1" ]; then
          quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobig_rot_${ib}_floatb_twist_matlab.m
        elif [ "$jacart" -eq "2" ]; then
          quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobia_rot_${ib}_floatb_twist_matlab.m
        elif [ "$jacart" -eq "3" ]; then
          quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobiR_rot_${ib}_floatb_twist_matlab.m
        elif [ "$jacart" -eq "4" ]; then
          quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobiRD_rot_${ib}_floatb_twist_matlab.m
        elif [ "$jacart" -eq "5" ]; then
          quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobia_transl_${ib}_floatb_twist_matlab.m
        elif [ "$jacart" -eq "6" ]; then
          quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobigD_rot_${ib}_floatb_twist_matlab.m
        elif [ "$jacart" -eq "7" ]; then
          quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobiaD_rot_${ib}_floatb_twist_matlab.m
        elif [ "$jacart" -eq "8" ]; then
          quelldat=$repo_pfad/codeexport/${robot_name}/tmp/jacobiaD_transl_${ib}_floatb_twist_matlab.m
        fi;
        # Einfügen von Quelltext
        if [ -f $quelldat ]; then
          echo "%% Variable Initialization" > ${quelldat}.subsvar
          cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
          if [ "$input_qD" == "true" ]; then
            cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
          fi;
          if [ "$input_r" == "true" ]; then
            printf "\npx = r_i_i_C(1);\npy = r_i_i_C(2);\npz = r_i_i_C(3);\n" >> ${quelldat}.subsvar
          fi;
          cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

          printf "\t%%%% Symbolic Calculation\n\t%% From ${quelldat##*/}\n" >> $zieldat
          sed -e 's/^/\t% /' ${quelldat}.stats >> $zieldat
           sed -e 's/^/\t/' $quelldat >> $zieldat
          source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
        else
          echo "Code in ${quelldat##*/} nicht gefunden."
          printf "\t%% Symbolic code from ${quelldat##*/} not found\n" >> $zieldat
          continue
        fi;
      done
      printf "end" >> $zieldat
    else # Funktionsaufruf bei den vollständigen Funktionen
      printf "\n%% Function calls\n" >> $zieldat
      if [ "$jacart" -eq "9" ]; then
        printf "Ja_transl = ${robot_name}_jacobia_transl_sym_varpar(qJ, link_index, r_i_i_C, ...\n  pkin);\n" >> $zieldat
        printf "Jg_rot = ${robot_name}_jacobig_rot_sym_varpar(qJ, link_index, ...\n  pkin);\n\n" >> $zieldat
        printf "Jg = [Ja_transl; Jg_rot];\nend" >> $zieldat
      elif [ "$jacart" -eq "10" ]; then
        printf "Ja_transl = ${robot_name}_jacobia_transl_sym_varpar(qJ, link_index, r_i_i_C, ...\n  pkin);\n" >> $zieldat
        printf "Ja_rot = ${robot_name}_jacobia_rot_sym_varpar(qJ, link_index, ...\n  pkin);\n\n" >> $zieldat
        printf "Ja = [Ja_transl; Ja_rot];\nend" >> $zieldat
      elif [ "$jacart" -eq "11" ]; then
        printf "JaD_transl = ${robot_name}_jacobiaD_transl_sym_varpar(qJ, qJD, link_index, r_i_i_C, ...\n  pkin);\n" >> $zieldat
        printf "JgD_rot = ${robot_name}_jacobigD_rot_sym_varpar(qJ, qJD, link_index, ...\n  pkin);\n\n" >> $zieldat
        printf "JgD = [JaD_transl; JgD_rot];\nend" >> $zieldat
      elif [ "$jacart" -eq "12" ]; then
        printf "JaD_transl = ${robot_name}_jacobiaD_transl_sym_varpar(qJ, qJD, link_index, r_i_i_C, ...\n  pkin);\n" >> $zieldat
        printf "JaD_rot = ${robot_name}_jacobiaD_rot_sym_varpar(qJ, qJD, link_index, ...\n  pkin);\n\n" >> $zieldat
        printf "JaD = [JaD_transl; JaD_rot];\nend" >> $zieldat
      fi;
    fi;
    source robot_codegen_matlabfcn_postprocess.sh $zieldat 0 0 ${quelldat}.subsvar
  done
fi;

# Funktionen für explizite kinematische Zwangsbedingungen
quelldat=$repo_pfad/codeexport/${robot_name}/tmp/kinconstr_expl_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_kinconstr_expl_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_kinconstr_expl.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
  source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 1 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

quelldat=$repo_pfad/codeexport/${robot_name}/tmp/kinconstr_expl_jacobian_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_kinconstr_expl_jacobian_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_kinconstr_expl_jacobian.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
  source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi

quelldat=$repo_pfad/codeexport/${robot_name}/tmp/kinconstr_expl_jacobianD_matlab.m
zieldat=$repo_pfad/codeexport/${robot_name}/matlabfcn/${robot_name}_kinconstr_expl_jacobianD_mdh_sym_varpar.m
if [ -f $quelldat ]; then
  cat $head_pfad/robot_matlabtmp_kinconstr_expl_jacobianD.head.m > $zieldat
  printf "%%%% Coder Information\n%%#codegen\n" >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 0
  source $repo_pfad/scripts/set_inputdim_line.sh $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJ.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_qJD.m >> $zieldat
  cat $tmp_pfad/robot_matlabtmp_assert_KP.m >> $zieldat

  echo "%% Variable Initialization" > ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_qJ.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_qJD.m >> ${quelldat}.subsvar
  cat $tmp_pfad/robot_matlabtmp_par_KP.m >> ${quelldat}.subsvar

  printf "\n%%%% Symbolic Calculation\n%% From ${quelldat##*/}\n" >> $zieldat
  sed -e 's/^/% /' ${quelldat}.stats >> $zieldat
  cat $quelldat >> $zieldat
  source robot_codegen_matlabfcn_postprocess.sh $zieldat 1 0 ${quelldat}.subsvar
else
  echo "Code in ${quelldat##*/} nicht gefunden."
fi
