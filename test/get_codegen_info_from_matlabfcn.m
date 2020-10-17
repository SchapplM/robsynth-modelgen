% Lese CodeGen-Statistik aus Matlab-Funktionen wieder aus.
% Statistik wird mit Maple-Prozedur "proc_MatlabExport" erzeugt

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-10
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function infostruct = get_codegen_info_from_matlabfcn(filepath)

% Form der Statistik:
infostruct = struct( ...
'OptimizationMode', NaN, ... % 0,1,2
'StartTime', 0, ... % Als Matlab-Zeitstempel (datenum)
'EndTime', 0, ...
'DurationCPUTime', NaN, ... % CPU-Zeit (gemessen von Maple) in Sekunden
'OptCodeLineCount', NaN, ... % Anzahl der Zeilen des optimierten Codes aus Maple
'OptCodeSize', NaN, ... % Anzahl der Zeichen (entspricht Bytes) des optimierten Codes
'FileSize', NaN, ... % Größe der Datei (Bytes)
... % Rechenaufwand, gemessen von Maple
'ComputationalCost', struct('add', 0, 'mult', 0, 'div', 0, 'fcn', 0, 'ass', 0), ...
... % Eigene Zählung der Rechenoperationen (nicht identisch zu Maple)
'ComputationalCostDebug', struct('add', 0, 'mult', 0, 'div', 0, 'fcn', 0, 'ass', 0));

% Ausdrücke aus dem Statistik-Block
expr1 = 'OptimizationMode: (\d)';
expr2 = 'StartTime: ([\d]+-[\d]+-[\d]+ [\d]+:[\d]+:[\d]+)';
expr3 = 'EndTime: ([\d]+-[\d]+-[\d]+ [\d]+:[\d]+:[\d]+)';
expr4 = 'DurationCPUTime: (\d)+.(\d)+s';
expr5 = 'Computational Cost: add\. \([\d]+->(\d)+\), mult\. \([\d]+->(\d)+\), div\. \([\d]+->(\d)+\), fcn\. \([\d]+->(\d)+\), ass\. \([\d]+->(\d)+\)';

fid = fopen(filepath, 'r');
if fid == -1
  warning('Datei %s konnte nicht geöffnet werden.', filepath);
  return
end
fileinfo = dir(filepath);
infostruct.FileSize = fileinfo.bytes;
tline = fgetl(fid);
expr_found = false(1,5);
linecount_gencode = 0;
charcount_gencode = 0;
i_statsblock = 1; % Es kann mehrere Blöcke mit codegen-stats geben
while ischar(tline) % Datei zeilenweise auslesen
  [tokens1,~] = regexp(tline,[expr1,'$'], 'tokens', 'match');
  if ~isempty(tokens1)
    infostruct.OptimizationMode(i_statsblock) = str2double(tokens1{1});
    expr_found(1) = true;
  end
  [tokens2,~] = regexp(tline,[expr2,'$'], 'tokens', 'match');
  if ~isempty(tokens2)
    infostruct.StartTime(i_statsblock) = datenum(tokens2{1});
    % check: datestr(infostruct.StartTime, 'yyyy-mm-dd HH:MM:SS') 
    expr_found(2) = true;
  end
  [tokens3,~] = regexp(tline,[expr3,'$'], 'tokens', 'match');
  if ~isempty(tokens3)
    infostruct.EndTime(i_statsblock) = datenum(tokens3{1});
    expr_found(3) = true;
  end
  [tokens4,~] = regexp(tline,[expr4,'$'], 'tokens', 'match');
  if ~isempty(tokens4)
    infostruct.DurationCPUTime(i_statsblock) = str2double([tokens4{1}{1},'.',tokens4{1}{2}]);
    expr_found(4) = true;
  end
  [tokens5,~] = regexp(tline,[expr5,'$'], 'tokens', 'match');
  if ~isempty(tokens5)
    infostruct.ComputationalCost.add(i_statsblock) = str2double(tokens5{1}{1});
    infostruct.ComputationalCost.mult(i_statsblock) = str2double(tokens5{1}{2});
    infostruct.ComputationalCost.div(i_statsblock) = str2double(tokens5{1}{3});
    infostruct.ComputationalCost.fcn(i_statsblock) = str2double(tokens5{1}{4});
    infostruct.ComputationalCost.ass(i_statsblock) = str2double(tokens5{1}{5});
    expr_found(5) = true;
  end
  if all(expr_found) && isempty(tokens5) % ist nicht mehr die Kommentar-Zeile
    % In Jacobi-Matrix-Funktion werden Blöcke so begrenzt:
    if ~isempty(regexp(tline, '%%', 'tokens')) || ...
        ~isempty(regexp(tline, 'elseif', 'tokens')) || ...
        ~isempty(regexp(tline, 'else', 'tokens'))
      expr_found = false(1,5); % Es kann nochmal neu geprüft werden, ob es ein Kommentarfeld mit stats gibt
      i_statsblock = i_statsblock + 1;
      tline = fgetl(fid); % nächste Zeile
      continue; % Neuer Abschnitt. Kein Auto-Code
    end
    % Variablen vergrößern, falls neuer Stats-Bereich gefunden
    if length(infostruct.ComputationalCostDebug.add) < i_statsblock
      infostruct.ComputationalCostDebug.add(i_statsblock) = 0;
    end
    if length(infostruct.ComputationalCostDebug.mult) < i_statsblock
      infostruct.ComputationalCostDebug.mult(i_statsblock) = 0;
    end
    if length(infostruct.ComputationalCostDebug.div) < i_statsblock
      infostruct.ComputationalCostDebug.div(i_statsblock) = 0;
    end
    if length(infostruct.ComputationalCostDebug.fcn) < i_statsblock
      infostruct.ComputationalCostDebug.fcn(i_statsblock) = 0;
    end
    if length(infostruct.ComputationalCostDebug.ass) < i_statsblock
      infostruct.ComputationalCostDebug.ass(i_statsblock) = 0;
    end
    if length(linecount_gencode) < i_statsblock
      linecount_gencode(i_statsblock) = 0; %#ok<AGROW>
    end
    if length(charcount_gencode) < i_statsblock
      charcount_gencode(i_statsblock) = 0; %#ok<AGROW>
    end
    
    % alle Ausdrücke vorher schon gefunden. Zähle nur noch die restlichen Code-Zeilen.
    linecount_gencode(i_statsblock) = linecount_gencode(i_statsblock) + 1; %#ok<AGROW>
    charcount_gencode(i_statsblock) = charcount_gencode(i_statsblock) + length(tline); %#ok<AGROW>
    % Zähle die Operatoren. Noch eher ungenau
    infostruct.ComputationalCostDebug.add(i_statsblock) = infostruct.ComputationalCostDebug.add(i_statsblock)  + ...
      sum(tline == '+') + sum(tline == '-');
    infostruct.ComputationalCostDebug.mult(i_statsblock) = infostruct.ComputationalCostDebug.mult(i_statsblock)  + ...
      sum(tline == '*');
    infostruct.ComputationalCostDebug.div(i_statsblock) = infostruct.ComputationalCostDebug.div(i_statsblock)  + ...
      sum(tline == '/');
    [tokens_sin, ~] = regexp(tline, '(sin\()', 'tokens', 'match');
    [tokens_cos, ~] = regexp(tline, '(cos\()', 'tokens', 'match');
    [tokens_atan, ~] = regexp(tline, '(atan2\()', 'tokens', 'match');
%     [tokens_square, ~] = regexp(tline, '(\^ 2)', 'tokens', 'match');
    infostruct.ComputationalCostDebug.fcn(i_statsblock) = infostruct.ComputationalCostDebug.fcn(i_statsblock)  + ...
      length(tokens_sin) + length(tokens_cos) + length(tokens_atan); %  + length(tokens_square)
    infostruct.ComputationalCostDebug.ass(i_statsblock) = infostruct.ComputationalCostDebug.ass(i_statsblock) + ...
      sum(tline == '=');
  end
  tline = fgetl(fid); % nächste Zeile
end
fclose(fid);
infostruct.OptCodeLineCount = linecount_gencode;
infostruct.OptCodeSize = charcount_gencode;