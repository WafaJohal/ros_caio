ROS_CAIO

Ros catkin repositories
copy the repo in your catkinws/src/
 be sure to have install swipl and that your python path is configured


TO BUILD : 
go in you catkin workspace

>cd /path/to/catkin_ws
> catkin_make
this generates the nodes in catkin_ws/devel

> source devel/setup.sh 

Now you should be able to see the packages for example
> rospack find appraisal_emo
your/path/catkin_ws/src/appraisal_emo
	Si vous avez : 
	[rospack] Error: package 'appraisal_emo' not found
	le source n'a pas fonctionner

Lancer roscore dans un nouveau terminal
> roscore 

pour lancer les noeuds 
> roslaunch caio.launch 
