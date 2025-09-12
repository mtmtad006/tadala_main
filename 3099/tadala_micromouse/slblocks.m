function blkStruct = slblocks

blkStruct.Name = 'UCT EEE libraries';

blkStruct.OpenFcn = {'mmuct_lib', 'dhaouadi2013_lib'}; % Opens specified libraries when the main entry is clicked

% Define browser entries for each library
Browser(1).Library = 'mmuct_lib';
Browser(1).Name    = 'UCT EEE Micromouse'; % Name displayed for Library1
Browser(1).IsFlat  = 0; % 0 for hierarchical library, 1 for flat

Browser(2).Library = 'dhaouadi2013_lib';
Browser(2).Name    = 'UCT dhaouadi2013'; % Name displayed for Library2
Browser(2).IsFlat  = 0;

Browser(3).Library = 'devel_lib';
Browser(3).Name    = 'UCT FN devel'; % Name displayed for Library2
Browser(3).IsFlat  = 0;

% Assign the browser structure to blkStruct
blkStruct.Browser = Browser;