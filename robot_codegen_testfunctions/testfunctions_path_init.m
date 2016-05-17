% Pfade hinzufügen
% 
% For this file to work, it has to be executed as a complete file, not
% line-wise (because of the function `mfilename`)
% 
% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-03
% (c) Institut für Regelungstechnik, Universität Hannover

%% Pfade hinzufügen

this_path = fileparts( mfilename('fullpath') );
addpath(this_path);

% Verzeichnis mit generierten Matlab-Funktionen hinzufügen
matlabfcn_path = fullfile(this_path, '../codeexport/matlabfcn');
addpath(matlabfcn_path);

simulink_path = fullfile(this_path, 'simulink');
addpath(simulink_path);
