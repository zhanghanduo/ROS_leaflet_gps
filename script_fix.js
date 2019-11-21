// ============================= INTRODUCTION

// Name : Autonomous-Car-GPS-Guiding
// Author : Sylvain ARNOUTS
// Mail : sylvain.ar@hotmail.fr
// Date : From May to August 2016
// Link : https://github.com/sylvainar/Autonomous-Car-GPS-Guiding

// This code has been written in order to create a nice interface to interact
// with the autonomous electric car of the UPV ai2 laboratory. It displays a map,
// set a marker on the car's position using gps publishing topic, and allows a user 
// to start the routing by tapping the destination on the touchscreen.

// License : Apache 2.0

// ============================= CONFIGURATION

// You can set here some parameters for the launch.
// In the lab we're using two different GPS, so further in the
// program, we're going to read rosparam to know what topic
// we should listen to and what is the number of cycles
// between each refreshing.

// The name of the GPS publisher name by default
var CONFIG_default_gps_topic_name = '/gps_bestpos'; // '/kitti/oxts/gps/fix';

var CONFIG_default_pose_topic_name = '/sslam/robot/pose';

// The number of cycles between every marker position reload
var CONFIG_cycles_number = 10;

// We can download the map online on OSM server, but
// it won't work if the car isn't connected to the internet.
// If you downloaded tiles and put it in the car, then you can
// access them in local, or else, connect to server.
// Set this config to "local" or "server".
var CONFIG_tile_source = 'server';

// If you use local tiles, set here the path to it
var CONFIG_tile_local_path = 'UPV/{z}/{x}/{y}.png';

// Network address to ROS server (it can be localhost or an IP)
var CONFIG_ROS_server_URI = 'localhost';

//===> Global variables
var map;
var selectionMode;
var carIcon = L.icon({
    iconUrl: 'assets/car1.png',
    iconSize: [22, 22],
    iconAnchor: [0, 0],
    popupAnchor: [-3, -30],
});
var markerPosition = L.marker([0,0]);
var markerPosition2 = L.marker([0,0], {icon: carIcon});
// ============================= FUNCTIONS
// ===> mapInit() : init the map
function mapInit() {
	
	//===> Var init

	// Fetch tiles
	if(CONFIG_tile_source == 'local')
		var tileUrl = CONFIG_tile_local_path;
	if(CONFIG_tile_source == 'server')
		var tileUrl = 'http://{s}.tile.osm.org/{z}/{x}/{y}.png';

	// Set attrib (always !)
	var attrib = 'Map data © OpenStreetMap contributors'; 

	//===> Map loading
	map = L.map('map');
	var osm = L.tileLayer(tileUrl, {
		minZoom: 11, 
		maxZoom: 19,
		attribution: attrib
	}); 
	osm.addTo(map);

	L.control.scale ({maxWidth:240, metric:true, imperial:false, position: 'bottomleft'}).addTo (map);
	let polylineMeasure = L.control.polylineMeasure ({
							position:'topleft', unit:'metres', showBearings:false, 
							clearMeasurementsOnStop: false, showClearControl: true})
	polylineMeasure.addTo (map);

	L.easyButton('glyphicon glyphicon-refresh', function(btn, map){
		window.location.reload();
	}).addTo(map);

	return map;
}

// ============================= SCRIPT
var bounds;
var currentPosition = {latitude : 0, longitude : 0};
var startPoint;
var endPoint;
var zoomLevel = 17;
var routeControl;
var loadedMap = false;
var loadedMap2 = false;
var i = 0;
var listenerGPS;
var listenerPose;

//===> ROS connexion
var ros = new ROSLIB.Ros({
	url : 'ws://'+ CONFIG_ROS_server_URI +':9090'
});

swal({
	title: "Connecting to ROS...",
	showConfirmButton: false,
	closeOnConfirm: false,
	showLoaderOnConfirm: true,
	allowOutsideClick: false,
	allowEscapeKey: false
});

ros.on('connection', function() {
	console.log('Connected to websocket server.');
	swal({
		title: "Waiting...",
		text: "The navigation module can't work without the GPS. Launch the GPS and the module will start automatically.",
		type: "info",
		confirmButtonText: "Reload",
		closeOnConfirm: false,
		allowOutsideClick: false,
		allowEscapeKey: false
	},
	function(){
		window.location.reload();
	});
});

ros.on('error', function(error) {
	console.log('Error connecting to websocket server: ', error);
	swal({
		title: "Error connecting the ROS server",
		text: "Unable to reach ROS server. Is rosbridge launched ?",
		type: "error",
		confirmButtonText: "Retry",
		closeOnConfirm: false,
		allowOutsideClick: false,
		allowEscapeKey: false
	},
	function(){
		window.location.reload();
	});
});

ros.on('close', function() {
	console.log("Connexion closed.");
	swal({
		title: "Error connecting the ROS server",
		text: "Unable to reach ROS server. Is rosbridge launched ?",
		type: "error",
		confirmButtonText: "Retry",
		closeOnConfirm: false,
		allowOutsideClick: false,
		allowEscapeKey: false
	},
	function(){
		window.location.reload();
	});
});

//===> Init the map
mapInit();

//===> Set the GPS listener
//  => Create param with initial value
var paramTopicNameValue = CONFIG_default_gps_topic_name;
var paramTopicNamePose = CONFIG_default_pose_topic_name;
var paramNbCyclesValue = CONFIG_cycles_number;

//  => Init the ROS param
var paramTopicName = new ROSLIB.Param({ros : ros, name : '/panel/gps_topic'});
var paramTopicPose_Name = new ROSLIB.Param({ros : ros, name : '/panel/pose_topic'});
var paramNbCycles = new ROSLIB.Param({ros : ros, name : '/panel/nb_cycles'});
var path_ = [];
var firstloc_x;
var firstloc_y;
var polyline_;
var polyline2_;

//  => Set the value
paramTopicName.get(function(value) { 
	// If the param isn't created yet, we keep the default value
	if(value != null) 
		paramTopicNameValue = value; 
	else
		paramTopicName.set(paramTopicNameValue);
	
	paramNbCycles.get(function(value) {
		// If the param isn't created yet, we keep the default value
		if(value != null) 
			paramNbCyclesValue = value; 
		else
		paramNbCycles.set(paramNbCyclesValue);


		// Set the listener information
		listenerGPS = new ROSLIB.Topic({
			ros : ros,
			name : paramTopicNameValue,
			// messageType : 'sensor_msgs/NavSatFix'
			messageType : 'rds_msgs/msg_novatel_bestpos'
		});

		// Set the callback function when a message from /gps is received

		var i = 0;

		listenerGPS.subscribe(function(message) {
			// We have to wait for the GPS before showing the map, because we don't know where we are
			var lat = message.latitude;
			var lon = message.longitude;

			if(loadedMap == false) 
			{
				swal.close();
				// Center the map on the car's position
				map.setView([lat, lon], zoomLevel);
				// Add the marker on the map
				markerPosition.addTo(map);
				markerPosition2.addTo(map);

				firstloc_x = message.northing;
				firstloc_y = message.easting;

				path_.push([lat, lon]);
				polyline_ = L.polyline(path_, {color: 'red'}, {weight: 1}).addTo(map);
				
				// Set the flag to true, so we don't have to load the map again
				loadedMap = true;
			}

			if(i % paramNbCyclesValue == 0)
			{
				// Refresh the global variable with the position
				currentPosition.latitude = lat;
				currentPosition.longitude = lon;
				// Refresh the position of the marker on the map
				markerPosition.setLatLng([lat, lon]);

				polyline_.addLatLng([lat, lon]);

				// console.log("path: ", JSON.stringify(path_));

				// If the marker has went out of the map, we move the map
				bounds = map.getBounds();
				if(!bounds.contains([lat, lon]))
					map.setView([lat, lon], zoomLevel);
				
				// console.log("Update position");
			}

			i++;

		});
	});
});

paramTopicPose_Name.get(function(value) { 
	// If the param isn't created yet, we keep the default value
	if(value != null)
		paramTopicNamePose = value; 
	else
		paramTopicPose_Name.set(paramTopicNamePose);

	paramNbCycles.get(function(value) {
		// If the param isn't created yet, we keep the default value
		if(value != null) 
			paramNbCyclesValue = value; 
		else
		paramNbCycles.set(paramNbCyclesValue);

		listenerPose = new ROSLIB.Topic({
			ros : ros,
			name : paramTopicNamePose,
			messageType : 'geometry_msgs/PoseWithCovarianceStamped'
		});

		var i2 = 0;
		var utm;

		listenerPose.subscribe(function(message2) {
			var x_ = message2.pose.pose.position.x + firstloc_x;
			var y_ = message2.pose.pose.position.z + firstloc_y;
			
			if(loadedMap2 == false) 
			{
				polyline2_ = L.polyline(path_, {color: 'blue'}).addTo(map);

				loadedMap2 = true;
			}

			if(i2 % paramNbCyclesValue == 0)
			{

				utm = L.utm(x_, y_, 48, 'N', false);
				var ll = utm.latLng();
				if (ll){
					markerPosition2.setLatLng(ll);
					polyline2_.addLatLng(ll);
				}

				console.log("Update position");

			}

			i2 ++;

		});



	});
});
