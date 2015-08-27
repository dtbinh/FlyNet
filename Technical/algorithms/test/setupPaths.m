setupPathsFileName = mfilename('fullpath');

testDirectory = fullfile(setupPathsFileName,'..','..','test');
srcDirectory = fullfile(setupPathsFileName,'..','..','source');

addpath(testDirectory);
addpath(srcDirectory);