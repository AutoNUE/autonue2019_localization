Submission Format:
=================

Format of d*.csv:
------------------
* column1: 'timestamp' (format-'HH-MM-SS-us')
 -Should contain the timestamp of the frame which are same as the one given in 
  the test.csv. 
* column2-4: 'tx,ty,tz'
 -Translation vector t=[tx,ty,tz] RELATIVE TO THE INITIAL POSITION.

NOTE :
1.WHEN SUBMITTING RESULTS,IN THE FOLLOWING FORMAT  USING THE FILE NAMES d0.csv 
  TO d2.csv. CREATE A ZIP ARCHIVE OF THEM AND STORE YOUR RESULTS IN ITS ROOT 
  FOLDER.


2.TIMESTAMPS MENTIONED IN THE SUBMISSION FILES SHOULD MATCH WITH THE RESPECTIVE
  TIMESTAMPS IN THE TEST.CSV FILE. SO YOU SHOULD HAVE A t([tx,ty,tz]) FOR EACH OF
  THE FRAME IN THE TEST DATA, i.e THERE ARE N TIMESTAMPS IN test.csv , THEN THE 
  SUBMISSION FILE SHOULD HAVE OUTPUT FOR THE SAME N TIMESTAMPS.

Evaluation Code:
================

For transperency we have included the IDD Localization challenge evaluation 
code. It can be compiled via:

"python idd_loc_eval.py [BASEDIR] [GTDIR]"

--BASEDIR contains the path to the directory which contains the final submission
  files d*.csv. 
--GTDIR contains the path to the directory which contains the ground truth files.

BASEDIR
  |---d0.csv
  |---d1.csv
  `---d2.csv

GTDIR 
  |---d0_gt.csv
  |---d1_gt.csv
  `---d2_gt.csv


* Format for d0.csv:
"
timestamp,tx,ty,tz
13-03-28-812057,0.0,0.0,0.0
13-03-28-888473,0.02,0.006,0.006
...
"

* Format for d0_gt.csv:
"
timestamp,latitude,longitude,altitude
13-03-25-812057,17.000000,78.000000,498.000000
13-03-25-888473,17.000001,78.000001,498.000001
....
"


NOTE:
1. In the evaluation code we have converted the groundtruth from gps to 
   cartesian coordinates using mercater's projection (with Radius of earth 
   r = 6378137 ).
2. The above formats are just an example and do not represent the actual 
   trajectory/ground truth.





