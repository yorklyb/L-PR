This is an insertable version of IILFM. https://github.com/York-SDCNLab/IILFM

How to use
***************************************************************************
mkdir build
mv detect_apriltag.py detect_aruco.py this.pcd ./build
cd build
cmake ..
make
./tag_detection
***************************************************************************
There are two parameters in the main.cpp file that you should pay attention to. 
The first one is the 'float m = 0.692; //marker_size in cm.' You should change the value according to your setup. 
The second one is 'system("python3 detect_aruco.py");' You should change the command according to your setup.

detect_aruco.py is for detecting ArUcos, and detect_apriltag.py is for detecting AprilTags. 
If you want to use other markers, you will need to write the Python script yourself.
