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

var gps_icon = L.icon({
    iconUrl: 'assets/gps.png',
    iconSize: [25, 25],
    iconAnchor: [10, 20],
    popupAnchor: [0, 0],
});

var estimation = L.icon({
    iconUrl: 'assets/perception.png',
    iconSize: [25, 25],
    iconAnchor: [10, 20],
    popupAnchor: [0, 0],
});
var carIcon = L.icon({
    iconUrl: 'assets/robot.png',
    iconSize: [26, 26],
    iconAnchor: [8, 25],
    popupAnchor: [0, 0],
});
var markerPosition = L.marker([0,0], {icon: gps_icon});
var markerPosition2 = L.marker([0,0], {icon: estimation});
var markerPosition3 = L.marker([0,0], {icon: carIcon});
var loadedMap = false;
var loadedMap2 = false;
var loadedMap3 = false;
// ============================= FUNCTIONS

// var rotateVector = function(vec, ang)
// {
//     // ang = -ang * (Math.PI/180);
//     var cos = Math.cos(ang);
//     var sin = Math.sin(ang);
//     return new Array(Math.round(10000*(vec[0] * cos - vec[1] * sin))/10000, Math.round(10000*(vec[0] * sin + vec[1] * cos))/10000);
// };

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
		loadedMap = false;
        loadedMap2 = false;
        loadedMap3 = false;
	}).addTo(map);

	return map;
}

// ============================= SCRIPT
var bounds;
var zoomLevel = 17;
var i = 0;
var listenerGPS;
var listenerPose;
var listernerOpt;
// If the input camera pose frame coordinate system is z_up or z_forward
var z_up = true;
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
var paramTopicNameOpt = '/pose_optimizer/optimized_pose';
var paramNbCyclesValue = CONFIG_cycles_number;

//  => Init the ROS param
var paramTopicName = new ROSLIB.Param({ros : ros, name : '/panel/gps_topic'});
var paramTopicPose_Name = new ROSLIB.Param({ros : ros, name : '/panel/pose_topic'});
var paramTopicOpt_Name = new ROSLIB.Param({ros : ros, name : '/panel/opt_topic'});
var paramNbCycles = new ROSLIB.Param({ros : ros, name : '/panel/nb_cycles'});
var path_ = [];
// var firstloc;
var polyline_;
var polyline2_;
var polyline3_;
var quaternion_;
// var rotation;
var translation_;

var quaternion0;
var translation0;

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
			quaternion_ = new THREE.Quaternion( message.pose.pose.orientation.x, 
													message.pose.pose.orientation.y, 
													message.pose.pose.orientation.z, 
													message.pose.pose.orientation.w );
			// rotation = new THREE.Euler().setFromQuaternion( quaternion_, 'XYZ' );

			translation_ = new THREE.Vector3( x_, y_, message.pose.pose.position.z );

			if(loadedMap == false)
			{
				swal.close();
				utm0 = L.utm(x_, y_, 48, 'N', false);
				var ll0 = utm0.latLng();
				if (ll0){
					map.setView(ll0, zoomLevel);
				}
                // console.log("gps first: ", x_, y_);
				// Add the marker on the map
				markerPosition.addTo(map);
				markerPosition2.addTo(map);
				markerPosition3.addTo(map);

				translation0 = translation_;
				quaternion0 = quaternion_;

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

			// 0 Declaration of vairables
			scale = new THREE.Vector3 (1, 1, 1);
			var cam2gps0 = new THREE.Matrix4();
			var quaternion_c = new THREE.Quaternion(message2.pose.pose.orientation.x,
													message2.pose.pose.orientation.y, 
													message2.pose.pose.orientation.z, 
													message2.pose.pose.orientation.w );

			var raw_vis = new THREE.Vector3( message2.pose.pose.position.x, message2.pose.pose.position.y, message2.pose.pose.position.z);

			var imu02enu = new THREE.Matrix4();

			if(z_up == true){
				cam2gps0.compose(raw_vis, quaternion_c, scale);

				// R(imu02enu) * R(imuk2imu0) = R(imuk2enu)
				// imu02enu.multiply(cam2gps0);

                cam2gps0.decompose(raw_vis, quaternion_c, scale);
                var x2_ = raw_vis.x + translation0.x;
                var y2_ = raw_vis.y + translation0.y;
			}
			else {
                imu02enu.compose(translation0, quaternion0, scale);

				var quaternion_cam2imu = new THREE.Quaternion( 0.5, -0.5, 0.5, -0.5 );
				var quaternion_imu2cam = new THREE.Quaternion( 0.5, -0.5, 0.5, 0.5 );
	
				var cam2imu = new THREE.Matrix4();
				cam2imu.makeRotationFromQuaternion(quaternion_cam2imu);
	
				// 1 R(camk2cam0) * R(imuk2camk) = R(imuk2cam0)
				quaternion_c.multiply(quaternion_imu2cam).normalize();
	
				cam2gps0.compose(raw_vis, quaternion_c, scale);
	
	
				// 2 R(imu02enu) * R(cam02imu0) * R(imuk2cam0) = R(imuk2enu)
	
				imu02enu.multiply(cam2imu);
	
				imu02enu.multiply(cam2gps0);

                imu02enu.decompose(raw_vis, quaternion_c, scale);

                var x2_ = raw_vis.x;
                var y2_ = raw_vis.y;
			}

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

paramTopicOpt_Name.get(function(value) { 
	// If the param isn't created yet, we keep the default value
	if(value != null)
		paramTopicNameOpt = value; 
	else
		paramTopicOpt_Name.set(paramTopicNameOpt);

	paramNbCycles.get(function(value) {
		// If the param isn't created yet, we keep the default value
		if(value != null) 
			paramNbCyclesValue = value; 
		else
		paramNbCycles.set(paramNbCyclesValue);

		listenerOpt = new ROSLIB.Topic({
			ros : ros,
			name : paramTopicNameOpt,
			messageType : 'geometry_msgs/PoseStamped'
		});

		var i3 = 0;
		var utm;

		listenerOpt.subscribe(function(message3) {

			// 0 Declaration of vairables
			scale = new THREE.Vector3 (1, 1, 1);
			var cam2gps0 = new THREE.Matrix4();
			var quaternion_c = new THREE.Quaternion(message3.pose.orientation.x,
													message3.pose.orientation.y,
													message3.pose.orientation.z,
													message3.pose.orientation.w );

			var raw_vis = new THREE.Vector3( message3.pose.position.x, message3.pose.position.y, message3.pose.position.z);

            var imu02enu = new THREE.Matrix4();

            if(z_up == true){
                cam2gps0.compose(raw_vis, quaternion_c, scale);

                // R(imu02enu) * R(imuk2imu0) = R(imuk2enu)
                // imu02enu.multiply(cam2gps0);

                cam2gps0.decompose(raw_vis, quaternion_c, scale);
                var x2_ = raw_vis.x + translation0.x;
                var y2_ = raw_vis.y + translation0.y;
            }
            else {
                imu02enu.compose(translation0, quaternion0, scale);

                var quaternion_cam2imu = new THREE.Quaternion( 0.5, -0.5, 0.5, -0.5 );
                var quaternion_imu2cam = new THREE.Quaternion( 0.5, -0.5, 0.5, 0.5 );

                var cam2imu = new THREE.Matrix4();
                cam2imu.makeRotationFromQuaternion(quaternion_cam2imu);

                // 1 R(camk2cam0) * R(imuk2camk) = R(imuk2cam0)
                quaternion_c.multiply(quaternion_imu2cam).normalize();

                cam2gps0.compose(raw_vis, quaternion_c, scale);


                // 2 R(imu02enu) * R(cam02imu0) * R(imuk2cam0) = R(imuk2enu)

                imu02enu.multiply(cam2imu);

                imu02enu.multiply(cam2gps0);

                imu02enu.decompose(raw_vis, quaternion_c, scale);

                var x2_ = raw_vis.x;
                var y2_ = raw_vis.y;
            }
			
			if(loadedMap3 == false) 
			{
				polyline3_ = L.polyline(path_, {color: 'green'}).addTo(map);

				loadedMap3 = true;
			}

			if(i3 % paramNbCyclesValue == 0)
			{

				utm = L.utm(x2_, y2_, 48, 'N', false);
				var ll = utm.latLng();
				if (ll){
					markerPosition3.setLatLng(ll);
					polyline3_.addLatLng(ll);
				}

				// console.log("Update position");

			}

			i3 ++;
		});
	});
});
