# ROS-LEAFLET-GPS

This application is designed to display an OSM map with Leaflet in a browser, but while being connected to ROS, so it can display the position and trajectory of the GPS using a marker. My next plan is to integrate visual odometry results in geometry_msgs/PoseStamped format.

## Installation 

This is not actually a ROS package, but it connects to ROS to get information from GPS sensor, and uses some ROS functionality, for instance ROS param, ROS topic, etc. 

To allow this application to connect with ROS, you need to install RosBridge, who is kind of a bridge to connect some Javascript apps to the ROS core. To do so, please follow the tutorial [here](http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge).

In order to launch rosbridge with the rest of your stuff, you can add this to your launch file :


    <launch>
        <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch" > 
            <arg name="port" value="9090"/>
        </include>
    </launch>

And then, open the page `index.html` with your favourite browser, and you're ready. You can even set the default page of your browser to it, so it opens automatically on startup.


## Configuration

The configuration of the module can be modified just by changing the value at the top of the script.js file.

- `CONFIG_tile_source` : Set the source of the tiles for the map. If you downloaded the maps of the area you want to move in, then you can set it to `local`. Else, set it to `server`.
- `CONFIG_tile_local_path` : Path to the downloaded tiles
- `CONFIG_ROS_server_URI` : Route to ROS server. It could be localhost or an IP.

## Parameters

In the lab, we're working with two differents GPS, which are not publishing on the same topic and at the same frequency. So two parameters can be set : 
- `/panel/gps_topic` for the GPS topic's name. Example : `/gps`
- `/panel/nb_cycles` for the number of cycles between each refreshing. Example : `20`

You can set those in a ROS launch file, or set it using `rosparam set`, then refreshing the page in the browser by clicking on the refresh button.
