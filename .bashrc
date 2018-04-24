#
#     ██████╗  █████╗ ███████╗██╗  ██╗██████╗  ██████╗
#     ██╔══██╗██╔══██╗██╔════╝██║  ██║██╔══██╗██╔════╝
#     ██████╔╝███████║███████╗███████║██████╔╝██║     
#     ██╔══██╗██╔══██║╚════██║██╔══██║██╔══██╗██║     
#  ██╗██████╔╝██║  ██║███████║██║  ██║██║  ██║╚██████╗
#  ╚═╝╚═════╝ ╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝ ╚═════╝
#                                                     
#
# 				- Andre Imperiali
#
#
#
#   __    __   
#  |_ /\ /  \. 
#  | /--\\_\/. 
#              
# What is the purpose of .bashrc and how does it work?
# https://unix.stackexchange.com/q/129143
#
#  _____   _____   _____   _____   _____   _____   _____  
#  \____\  \____\  \____\  \____\  \____\  \____\  \____\ 
#     _____      _               
#    / ____|    | |              
#   | (___   ___| |_ _   _ _ __  
#    \___ \ / _ \ __| | | | '_ \ 
#    ____) |  __/ |_| |_| | |_) |
#   |_____/ \___|\__|\__,_| .__/ 
#                         | |    
#                         |_|                          
#   __  __  __ 
#  |__)/  \(_  
#  | \ \__/__) 
#              
source /opt/ros/kinetic/setup.bash
source /adhome/aimperiali/catkin_ws/devel/setup.bash
#  ___                     
#   | _ _ _ . _  _ |_ _  _ 
#   |(-| ||||| )(_||_(_)|  
#                          
export PS1="\[\033[38;5;11m\]\u\[$(tput sgr0)\]\[\033[38;5;15m\]@\h:\[$(tput sgr0)\]\[\033[38;5;6m\][\w]:\[$(tput sgr0)\]\[\033[38;5;15m\] \[$(tput sgr0)\]"
#              __                   
#  |   _ |_   /   _  _  _    |_ _ _ 
#  |__(_||_)  \__(_)||||_)|_||_(-|  
#                      |            
#export ROS_HOSTNAME=ak120d-02.wpi.edu # <----- No clue
export ROS_HOSTNAME=ak120d-05.wpi.edu # <----- Favorite desk
#export ROS_HOSTNAME=ak120d-13.wpi.edu # <----- No clue
#export ROS_HOSTNAME= 'echo $HOSTNAME'.wpi.edu # Attempted to figure out the hostname
#  ___                   
#   |     _|_| _|_  _ |_ 
#   | |_|| |_|(-|_)(_)|_ 
#                        
export TURTLEBOT3_MODEL=burger
#  ___        __            
#   | _  |_  |_  _|.|_ _  _ 
#   |(-)(|_  |__(_|||_(_)|  
#                           
export EDITOR="atom"
export VISUAL="atom"
#
#  _____   _____   _____   _____   _____   _____   _____  
#  \____\  \____\  \____\  \____\  \____\  \____\  \____\ 
#             _ _                     
#       /\   | (_)                    
#      /  \  | |_  __ _ ___  ___  ___ 
#     / /\ \ | | |/ _` / __|/ _ \/ __|
#    / ____ \| | | (_| \__ \  __/\__ \
#   /_/    \_\_|_|\__,_|___/\___||___/
#                                     
#                                     
#   __         __ 
#  |__) \  / |  / 
#  |  \  \/  | /_ 
#                 
alias lab3_rviz="rosrun rviz rviz -d `rospack find rbe3002Lab3`/rviz/Lab3Maze.rviz"
alias lab4_rviz="rosrun rviz rviz -d `rospack find rbe3002lab4`/rviz/Lab4Maze.rviz"
#   __       __  ___  __   __  
#  / _`  /\   / |__  |__) /  \ 
#  \__> /~~\ /_ |___ |__) \__/ 
#                              
alias gzb="roslaunch turtlebot3_gazebo turtlebot3_world.launch"	
alias rvz="roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch"
#              __   __  
#   |\/|  /\  |__) /__` 
#   |  | /~~\ |    .__/ 
#                       
alias MAP3_simplemap="rosrun map_server map_server `rospack find rbe3002Lab3`/stagemaps/simple_map.yaml"
alias MAP4_Lab-4="rosrun map_server map_server `rospack find rbe3002lab4`/stagemaps/turtlebot3_world.yaml"
alias MAP5_Lab-4="roslaunch turtlebot3_navigation turtlebot3_navigation.launch
map_file:=`rospack find rbe3002lab4`/stagemaps/turtlebot3_world.yaml"
alias MAPTest="rosrun map_server map_server `rospack find rbe3002_d2018_final_gazebo`/maps/maze2.yaml"

alias MAP6_Lab-4="roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=`rospack find rbe3002lab4`/stagemaps/turtlebot3_world.yaml"

#                       __       
#  |     /\  |  | |\ | /  ` |__| 
#  |___ /~~\ \__/ | \| \__, |  | 
#                                
alias labrun3="rosrun rbe3002lab3 RBE3002_2018D_Lab3_Grid_Cells.py"
alias labrun4="rosrun rbe3002lab4 lab4.py"
alias primetest="rosrun rbe3002lab4 primalityTest.py"
#   __        __   __  ___                  ___  __  
#  /__` |__| /  \ |__)  |  |\ |  /\   |\/| |__  /__` 
#  .__/ |  | \__/ |  \  |  | \| /~~\  |  | |___ .__/ 
#                                                    
alias mr4=MAP4_Lab-4
alias mg4=MAP5_Lab-4
alias r4=lab4_rviz
alias g4=gzb
alias l3=labrun3
alias l4=labrun4

alias fin="rosrun rbe3002_d2018_final_gazebo final.py"

alias c3="roslaunch rbe3002_d2018_final_gazebo practice_exploration.launch"
alias b2="roslaunch rbe3002_d2018_final_gazebo maze_sim.launch"
alias a1="roslaunch rbe3002_d2018_final_gazebo demo_navigation.launch"
alias d4="roslaunch rbe3002_d2018_final_gazebo practice_maze.launch"

#d4 & a1

# Terminator
#alias CDCATKIN="cd catkin_ws/"
#alias CDPROJECTS="cd catkin_ws/src/"
#alias OPENATOM="atom . ./src/rbe3002lab4/scripts/lab4.py"

# Important
alias gazeebs="roslaunch rbe3002_d2018_final_gazebo practice_maze.launch"
alias themap="roslaunch rbe3002_d2018_final_gazebo practice_exploration.launch"
alias finalrviz="rosrun rviz rviz -d `rospack find rbe3002_d2018_final_gazebo`/rviz/final.rviz"


#
#  _____   _____   _____   _____   _____   _____   _____  
#  \____\  \____\  \____\  \____\  \____\  \____\  \____\ 
#     _____                                          _     
#    / ____|                                        | |    
#   | |     ___  _ __ ___  _ __ ___   __ _ _ __   __| |___ 
#   | |    / _ \| '_ ` _ \| '_ ` _ \ / _` | '_ \ / _` / __|
#   | |___| (_) | | | | | | | | | | | (_| | | | | (_| \__ \
#    \_____\___/|_| |_| |_|_| |_| |_|\__,_|_| |_|\__,_|___/
#                                                          
#                                                          
#   __     
#  / _ .|_ 
#  \__)||_ 
#          
cmt() { "git" "commit" "-a" "-m" "$1"; }
psh() { "git" "push"; }
brch() { "git" "branch"; }
pll() { "git" "pull"; }
sts() { "git" "status"; }

CDCATKIN() { "cd" "catkin_ws/"; }
CDPROJECTS() { "cd" "catkin_ws/src/"; }
OPENATOM() { "atom" "." "./src/rbe3002lab4/scripts/lab4.py"; }
brc() { "gedit" ".bashrc"; }



prnt0() { "echo" " - Lab 4 Commands - "; }
prnt1() { "echo" "mr4: 2D Map - rviz"; }
prnt2() { "echo" "mg4: 3D Map - gazebo"; }
prnt3() { "echo" "r4: rviz"; }
prnt4() { "echo" "l4: Code"; }
prnt01() { "echo" " - IN CASE ROS BREAKS - "; }
prnt5() { "echo" "killall -9 roscore"; }
prnt6() { "echo" "killall -9 rosmaster"; }
prntline() { "echo" "- - - - - - - - -"; }


