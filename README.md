# 202020-spring-2020-projects-afs-2
* Object Avoidance System
  1. Object detection/tracking using HSV.
  2. Avoidance Algorithm set it place.
  
 
# Dodging a yellow object while moving forward
1. The drone will move forward in a controlled environment in which a yellow object will move into its path
2. Once a yellow object moves into the drone's path, the hsv_edge function in main.py will confirm that the object is yellow.
3. Once a yellow object is found, the aviodance algorithm will draw a box around the object.
4. The box will allow the dodge to be performed by calculating the area and position then attempt to dodge but only if the object is in the path of the drone.
5. The objects position in relation to the drone will be based on x and y cordinates on the camera view and area of drawnn box around object.
6. Once all that is confirmed, the drone will commence dodging. Dodging entails moving away from object then continue forward.

# Current Implementation 
This is an implementation of object tracking on the dji Tello drone based on HSV Edging using OpenCV and Python 3.6 on visual studio code.

The current implementation allows a user to:

* Test other dodging methods using 'testing_OA.py'
* Launch the drone using command in terminal 'python main.py'
* View both the video feed from the drone to the computer and the HSV edging used by the drone.

It allows the drone to:

* Move in a forward path, while detecting objects.
* Detect object of the color yellow.
* If hsv edging detects a yellow object, drone will use avoidance algorithm to dodge objects in its path.

**Note:** Current implementation allows only one yellow object dodge at a time. Also a dodge and recenter movement.

**Warning!!** 
We are aware of an issue when initially installing imports. The issue revolves around opencv and visual studio code.
1. Open up the 'settings.json' file in visual studio code.
2. The code below should be put in the json file (autosave is just a personal preference).
```bash
  {
    "files.autoSave": "off",
    "python.linting.pylintArgs": ["--generate-members"],
    
  }
```
3. If that fix does not work then there might be an issue with your path, as discoverd by a team member.

# Installs via pip
1. Starts with scip.spatial, numpy, and others. THe others may be excluded, but I kept for possible future work.
```bash
python -m pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose
```
2. Next up is opencv.
```bash
pip install --user opencv-contrib-python
```
3 If any missing import errors start popping up check libraries of pip installs with command below.
```bash
pip list
```

# Avoiding a yellow in direct path of drone
1. Requires a solid yellow object.
2. Drone must connect with the computer via wifi.
3. Open terminal and enter command below.
```bash
python main.py
```
4. Drone will launch and start moving forward detecting objects.
5. If a yellow object is determined to be in the drone's path it will perform a dodge/recenter and on the camera view, red text 'dodge!!!' will display.
6. To stop the drone, the 'q' key must be pressed.

# Have fun testing your own methods
1. Same conditions necessary for 'main.py' must be met by 'testing_OA.py'
2. There will be a 'testing_OA.py' file for testing any new methods you might have.
3. This will allow you or anyone to build a unique Avoidance Algorithm.
4. Running, will require the command below.
```bash
python testing_OA.py
```
5. If you manage to getting a better working algorithm add to 'main.py'
# Handling both Avodiance systems
1. The file 'first_OA.py' has no known issues. However, it does only performs a single movement dodge.
2. The file 'second_OA.py' is the current setup for 'main.py' . The problem with this method is they delay factor that was added to sending the drone commands in a timely manner. The problem with this is that an object could enter frame during this delay an cause collision. However this is a controlled environment so it works accordingly. 
# It just won't work
1. Not every line of code works, that leaves both broken programs and dysfunctional code in your way.
2. The file 'broken_code.py' will allow you to throw away any code that hasn't worked, without deleting it.
3. It will clear up any commented out code that you have.
4. It will allow you to be able to go back to code that may work with the new implementation.
5. The file 'broken_code.py' currently has my broken code.
6. Try not to make too many new implementations to this file.
7. No need to execute this file, since there might be a **very slight** chance it is broken.

# Source Links
* This link takes you to the file that both the HSV edging and coordinate system were pulled from: https://github.com/DrexelLagare/Senior-Design-1/blob/master/Object%20Tracking/target_A.py
* This link takes you to the file that the tello camera feed was pulled from: https://github.com/DrexelLagare/Senior-Design-1/blob/master/face_recognition/tello_drone.py