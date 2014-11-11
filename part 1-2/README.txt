How to Run

To run the code, go into the src directory and run the MATLAB function 
RunBug

The first parameter is the domain file. The available domain files here are
bugDomain1.txt, bugDomain2.txt and localMinDomain.txt.

The second argument is the algorithm. 1 is bug 1, 2 is bug 2 and 3 is 
potential fields

Note - Code is made to work with an arbitrary number of circular obstacles 
or one polygon obstacle. Potential fields is not guaranteed to work with
polygon obstacle files. This code has not been thoroughly debugged for
other possible obstacle files, but it gets the correct results for this
project (without hardcoding).