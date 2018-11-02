// ============================= INTRODUCTION

// Name : Leaflet mapping for ROS
// Author : Zhang Handuo
// Mail : hzhang032@e.ntu.edu.sg
// Date : November 2018
// Link : https://github.com/zhanghanduo/ROS_leaflet_gps

// License : Apache 2.0

// ============================= CONFIGURATION

// You can set here some parameters for the launch.
// In the lab we're using two different GPS, so further in the
// program, we're going to read rosparam to know what topic
// we should listen to and what is the number of cycles
// between each refreshing.

// The name of the GPS publisher name by default
var CONFIG_default_gps_topic_name = '/gps_pose'; // '/kitti/oxts/gps/fix';

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
    iconAnchor: [10, 20],
    popupAnchor: [0, 0],
});
var markerPosition = L.marker([0,0]);
var markerPosition2 = L.marker([0,0], {icon: carIcon});
// ============================= FUNCTIONS

var rotateVector = function(vec, ang)
{
    // ang = -ang * (Math.PI/180);
    var cos = Math.cos(ang);
    var sin = Math.sin(ang);
    return new Array(Math.round(10000*(vec[0] * cos - vec[1] * sin))/10000, Math.round(10000*(vec[0] * sin + vec[1] * cos))/10000);
};

// // Rotate an object around an axis in object space
// function rotateAroundObjectAxis( object, axis, radians ) {

//     var rotationMatrix = new THREE.Matrix4();

//     rotationMatrix.makeRotationAxis( axis.normalize(), radians );
//     object.matrix.multiplySelf( rotationMatrix );                       // post-multiply
//     object.rotation.setRotationFromMatrix( object.matrix );
// }

// // Rotate an object around an axis in world space (the axis passes through the object's position)       
// function rotateAroundWorldAxis( object, axis, radians ) {

//     var rotationMatrix = new THREE.Matrix4();

//     rotationMatrix.makeRotationAxis( axis.normalize(), radians );
//     rotationMatrix.multiplySelf( object.matrix );                       // pre-multiply
//     object.matrix = rotationMatrix;
//     object.rotation.setRotationFromMatrix( object.matrix );
// }

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


	map.addControl(new L.Control.LinearMeasurement({
		unitSystem: 'metric',
		color: '#FF0080'
	}));

	L.easyButton('glyphicon glyphicon-refresh', function(btn, map){
		window.location.reload();
	}).addTo(map);

	return map;
}

// ============================= SCRIPT
var bounds;
var zoomLevel = 17;
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
var firstloc;
var polyline_;
var polyline2_;
var rot;

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
			messageType : 'geometry_msgs/PoseWithCovarianceStamped'
		});

		// Set the callback function when a message from /gps is received

		var i = 0;
		var utm0;

		listenerGPS.subscribe(function(message) {
			// We have to wait for the GPS before showing the map, because we don't know where we are
			var x_ = message.pose.pose.position.x;
			var y_ = message.pose.pose.position.y;
			var quaternion = new THREE.Quaternion(message.pose.pose.orientation.x, 
													message.pose.pose.orientation.y, 
													message.pose.pose.orientation.z, 
													message.pose.pose.orientation.w );
			var rotation = new THREE.Euler().setFromQuaternion( quaternion, 'XYZ' );
			rot = Math.PI/2 - rotation.z;
			// console.log("rotation: ", rot);

			if(loadedMap == false)
			{
				swal.close();
				utm0 = L.utm(x_, y_, 48, 'N', false);
				var ll0 = utm0.latLng();
				if (ll0){
					map.setView(ll0, zoomLevel);
				}
				// Add the marker on the map
				markerPosition.addTo(map);
				markerPosition2.addTo(map);

				firstloc = [x_, y_];

				path_.push(ll0);
				polyline_ = L.polyline(path_, {color: 'red'}, {weight: 1}).addTo(map);
				
				// Set the flag to true, so we don't have to load the map again
				loadedMap = true;
			}

			if(i % paramNbCyclesValue == 0)
			{

				utm0 = L.utm(x_, y_, 48, 'N', false);
				ll0 = utm0.latLng();
				if (ll0){
					markerPosition.setLatLng(ll0);
					polyline_.addLatLng(ll0);
				}

				// console.log("path: ", JSON.stringify(path_));

				// If the marker has went out of the map, we move the map
				bounds = map.getBounds();
				if(!bounds.contains(ll0))
					map.setView(ll0, zoomLevel);
				
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


			var raw_vis = [ message2.pose.pose.position.x, message2.pose.pose.position.z ];
			var after_rot = rotateVector(raw_vis, - rot);
			var x2_ = firstloc[0] + after_rot[0];
			var y2_ = firstloc[1] + after_rot[1];


			// var raw_vis = new THREE.Vector3( message2.pose.pose.position.x, message2.pose.pose.position.z, 0);
			// var axis_ = new THREE.Vector3( 0, 0, 1);
			// var rotationMatrix = new THREE.Matrix4(); 
			// raw_vis.applyMatrix4(rotationMatrix.makeRotationAxis( axis_, - rot ));
			// var x2_ = firstloc[0] + raw_vis.x;
			// var y2_ = firstloc[1] + raw_vis.y;
			
			if(loadedMap2 == false) 
			{
				polyline2_ = L.polyline(path_, {color: 'blue'}).addTo(map);

				loadedMap2 = true;
			}

			if(i2 % paramNbCyclesValue == 0)
			{

				utm = L.utm(x2_, y2_, 48, 'N', false);
				var ll = utm.latLng();
				if (ll){
					markerPosition2.setLatLng(ll);
					polyline2_.addLatLng(ll);
				}

				// console.log("Update position");

			}

			i2 ++;
		});
	});
});
