# ROS-OSM-map-integration

This application is designed to display an OSM map with Leaflet in a browser, but while being connected to ROS, so it can display the position of the bot using a marker. You can also tap for a destination on the screen, and the app will set the position wanted in the ROS params. To make the routing from your current position to the point you've clicked on, please refer to my ROS module [Routing-ROS-Module](https://github.com/sylvainar/Routing-ROS-Module).

![Image](screenshot.png)

*A screenshot and the integration of the module on a car (Lab ai2, UPV)*

## Installation 

This is not actually a ROS package, but it connects to ROS to get informations from GPS sensor, and uses some ROS functionnality, for instance ROS param, ROS topic, etc. 

To allow this application to connect with ROS, you need to install RosBridge, who is kind of a bridge to connect some Javascript apps to the ROS core. To do so, please follow the tutorial [here](http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge).

In order to launch rosbridge with the rest of your stuff, you can add this to your launch file :


    <launch>
        <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch" > 
            <arg name="port" value="8080"/>
        </include>
    </launch>

And then, open the page `index.html` with your favourite browser, and you're ready. You can even set the default page of your browser to it, so it opens automatically on startup.


## Configuration

The configuration of the module can be modified just by changing the value at the top of the script.js file.

- `CONFIG_default_gps_topic_name` : Set the name of the GPS topic the app is going to listen to
- `CONFIG_cycles_number` : If the GPS is publishing really really fast, the app doesn't have the time to update the marker at each cycle, causing some delay. This parameters sets the number of cycles between each actualisation.
- `CONFIG_tile_source` : Set the source of the tiles for the map. If you downloaded the maps of the area you want to move in, then you can set it to `local`. Else, set it to `server`.
- `CONFIG_tile_local_path` : Path to the downloaded tiles
- `CONFIG_ROS_server_URI` : Route to ROS server. It could be localhost or an IP.

## Parameters

In the lab, we're working with two differents GPS, which are not publishing on the same topic and at the same frequency. So two parameters can be set : 
- `/panel/gps_topic` for the GPS topic's name. Example : `/gps`
- `/panel/nb_cycles` for the number of cycles between each refreshing. Example : `20`

You can set those in a ROS launch file, or set it using `rosparam set`, then refreshing the page in the browser by clicking on the refresh button.
