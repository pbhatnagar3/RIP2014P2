README

Commands:
4. a) RRT([]);
      RRT([0, 1.3, 1.57, 2, 5]);
   b) RRTGoal([0.14;1.45;1.0], [-2.5;-0.5;-2.1], []);
      RRTGoal([0.14;1.45;1.0], [-2.5;-0.5;-2.1], [-0.7, 1.3, 1.57, 2, 5;4.0, -0.9, 0, 2, 1;-2.6, -0.7, 0, 1, 1]);
   c) RRTConnect([0.14;1.45;1.0], [-2.5;-0.5;-2.1], []);
      RRTConnect([0.14;1.45;1.0], [-2.5;-0.5;-2.1], [-0.7, 1.3, 1.57, 2, 5;4.0, -0.9, 0, 2, 1;-2.6, -0.7, 0, 1, 1]);
   d) RRTBidir([0.14;1.45;1.0], [-2.5;-0.5;-2.1], []);
      RRTBidir([0.14;1.45;1.0], [-2.5;-0.5;-2.1], [-0.7, 1.3, 1.57, 2, 5;4.0, -0.9, 0, 2, 1;-2.6, -0.7, 0, 1, 1]);
   
To output video files, uncomment lines using the command "writeVideoFile". There should be two per file (one for tree, one for arm). 