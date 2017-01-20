function outVec = struct2vec(inStruct, field)

    T = struct2table(inStruct);
    
    indx = find(strcmp(T.Properties.VariableNames, field));
    
    outVec = table2array(T(:, indx));

end