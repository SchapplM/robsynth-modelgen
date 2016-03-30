% Compile all Functions as mex to check if asserts are correct

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-01
% (c) Institut für Regelungstechnik, Universität Hannover

clear
clc

%% Init
testfunctions_path_init
compile_path = matlabfcn_path; % aus testfunctions_path_init

%% Loop through files and compile all Matlab Functions
for p = {compile_path}
  % suche m-Dateien
  files = dir(fullfile(p{1}, '*.m'));
  for i = 1:length(files)
    fprintf('%03d: %s\n', i, files(i).name);
    % Prüfe ob Datei "%#codegen" enthält
    fid = fopen(fullfile(p{1}, files(i).name),'r');
    tline = fgetl(fid);
    codegen_set = false;
    while ischar(tline)
      if strfind(tline, '%#codegen')
        codegen_set = true;
        break;
      end
      tline = fgetl(fid);
    end
    fclose(fid);
    if ~codegen_set
      fprintf('No %%#codegen - tag in file. Skip.\n');
      continue
    end
    % einzelne Datei kompilieren
    [~,filebasename,~]=fileparts(files(i).name);
    
    Fehler = Mex_Erstellen({filebasename}, false, false, false);

    if Fehler
      error('Fehler');
    end
  end
end

fprintf('Alle m-Funktionen als mex kompiliert!\n');