HOW TO NAME LOG FILES AND WHERE TO PUT THEM===================================
1) Go into the folder (CURRENT_YEAR)/(QUARTER)/
2) Name the folder containing the log files in the following form:
	name << month << '-' << day << '_' << keykword1 << '_' << keyword2
	The more keywords the better, put the most descriptive one first
	Don't worry about naming individual log files, unless there was
	something important in one of them
3) Done


ABOUT MATLAB SCRIPTS==========================================================

The Matlab log analysis scripts, e.g., Log_Analysis.m, simple_kalman/main.m
all work in generally the same way. 

How to run scripts:

 1. Open the script up in Matlab or Octave.

 2. There should exist a line near the top of the script which reads something
    much like:
       log = load('RunFilterTestLog.TXT');
    This line loads the log file

 3. Edit the text inside load() to the log you want to analyze. e.g., if you 
    want to analyze ~/yolo/swag.TXT, you would edit the line to say
       log = load('~/yolo/swag.TXT');
 
 4. Run the script.

Notes on the scripts:
 
 * Scripts are usually written to work with the log format version that was on
   the vehicle during when they were written.

 * Scripts are forward compatible. Older scripts generally work on newer log
   formats because we add fields to the log at the end and don't remove
   fields.

 * Scripts may or may not be backwards compatible as older log formats may not
   include all the information that is needed by the script.
