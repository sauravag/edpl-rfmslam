function initialGuessTraj = load_initial_guess_from_graph(fname)
% open the file
fid = fopen(fname);

initialGuessTraj = [];

% read it line by line
while true
    
    % read line
    tline = fgetl(fid);
    
    % check if it has anything in it
    if ~ischar(tline);
        break;%end of file
    end
    
    % if it had something, lets see what it had
    % Scan the line upto the first space
    % we will identify what data type this is
    str = textscan(tline,'%s %*[ ]');
    
    if strcmp(str{1},'VERTEX2')
        
        datastr = textscan(tline,'%s %d %f %f %f');        
        
        initialGuessTraj = [initialGuessTraj, [datastr{3};datastr{4};datastr{5}]];
            
    end
    
end

% close file
fclose(fid);
end