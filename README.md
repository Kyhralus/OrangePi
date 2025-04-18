# Auto-navigate based on ros2 and navigation2 
*This is the test of git.
But the project is finished!*
[alineyiee@163.com]
(hhhha)
- p1
- p2
- p3
```
. install/
```
## Project introduction

## Project build steps

## References

## The use steps
1. build the project
```
cd navigation2_ws
colcon build
```
2. [all Terminates] setup the env.   # You need to run the following command on each terminal!!!
```
. install/setup.bash
```
3. [Terminate1] launch the map_server
```
ros2 launch mybot_description mybot_description.launch.py
```
4. [Terminate2] launch the lidar node --- which is NOT IN MY PROJECT !!!, you should choose one by yourself
THIS IS MY OWN LIDAR PROGRAMMER!!!
```
cd lanhailidar_ros_ws
. install/setup.bash
ros2 launch bluesea2 uart_lidar.launch    # launch the executable file
``` 
5. [Terminate3] launch the <ros2_laser_scan_mathcer> packages
```
ros2 run ros2_laser_scan_matcher laser_scan_matcher
```
6. [Terminate4] launch the main launch file. YOU`D BETTER LAUNCH THIS COMMAND IN MOBAXTERM
```
ros2 launch mybot_navigation2 mybot_nav2.launch.py
```


# GIT steps
1. initialize the repository
```
cd /navigation2_ws
git init
```
2. (select)*to ingnore some files you don't want to pull
```
mkdir .gitignore
vim .gitignore     # add the unnecessary files' suffixes
``` 
3. add to Staging Area
```
git add .     # add ALL files to Staging Area
              # git add <file_name>     you can use this command to add specified files
```
4. commit your codes to Local Repository
```
git commit -m "commit message"  
```
5. check the status of files
``` 
git status                          
```
6. git to github
```
git push -u origin main # Upload to the main branch of the GitHub repository corresponding to Origin
```
7. view the git log
```
git log
```
