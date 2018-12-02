function T = load_csv( file )
%LOAD_DATA reads table in a .csv file at TCC\data\...
T = readtable(file, 'Delimiter', ',');
for k = T.Properties.VariableNames
    x = T.(k{1});
    M = [];
    if iscell(x)
        for i = 1:length(x)    
            if x{i}(1) == '[' && x{i}(end) == ']'
                x{i} = textscan(x{i}(2:end-1), '%f');
            end
            M(i, :) = x{i}{1}';
        end
    else
        M = x;
    end
    T.(k{1}) = M;
end
end