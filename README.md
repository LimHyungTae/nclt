# NCLT

*** 1. Undistort image 

nclt_tools/distort_all_ours.py
You need U2D_Cam5_1616X1232.txt
Cam5 represents frontal camera

*** 2. Synced pose 

nclt_tools/write_sync_pose.py

*** 3. Parse Hokuyo 

nclt_tools/read_hokuyo_30m.py

** Notice
tf: consists of 3x4 because 0,0,0,1 in the last row is overlapped!
Hokuyo: consists of 2x1081 only consists of x and y
