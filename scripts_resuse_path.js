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

var CONFIG_default_pose_topic_name = '/loop_fusion_node/pose_graph_path';

// The number of cycles between every marker position reload
var CONFIG_cycles_number = 10;

// If the input camera pose frame coordinate system is z_up or z_forward
var z_up = true;

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


	// map.addControl(new L.Control.LinearMeasurement({
	// 	unitSystem: 'metric',
	// 	color: '#FF0080'
	// }));

	L.control.scale ({maxWidth:240, metric:true, imperial:false, position: 'bottomleft'}).addTo (map);
	let polylineMeasure = L.control.polylineMeasure ({
							position:'topleft', unit:'metres', showBearings:false, 
							clearMeasurementsOnStop: false, showClearControl: true})
	polylineMeasure.addTo (map);

	L.easyButton('glyphicon glyphicon-refresh', function(btn, map){
		window.location.reload();
		loadedMap = false;
        loadedMap2 = false;
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
// z_up = new ROSLIB.Param({ros : ros, name : 'z_up'});
console.log("z_up?: ", z_up);
var path_ = [];
// var firstloc;
var polyline_;
var polyline2_;
var quaternion_;
var translation_;

var quaternion0;
var translation0;
var imu02enu = new THREE.Matrix4();
var scale = new THREE.Vector3 (1, 1, 1);
var quaternion_cam2imu = new THREE.Quaternion( 0.5, -0.5, 0.5, -0.5 );
var quaternion_imu2cam = new THREE.Quaternion( 0.5, -0.5, 0.5, 0.5 );
var cam2imu = new THREE.Matrix4();
cam2imu.makeRotationFromQuaternion(quaternion_cam2imu);

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

		var i1 = 0;
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
			// rot = rotation.z - Math.PI/4;
			// console.log("rotation: ", rot);
			translation_ = new THREE.Vector3( x_, y_, message.pose.pose.position.z );

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

				// firstloc = [x_, y_];
				translation0 = translation_;
				quaternion0 = quaternion_;
				imu02enu.compose(translation0, quaternion0, scale);

				path_.push(ll0);
				polyline_ = L.polyline(path_, {color: 'red'}, {weight: 1}).addTo(map);
				
				// Set the flag to true, so we don't have to load the map again
				loadedMap = true;
			}

			if(i1 % paramNbCyclesValue == 0)
			{

				utm0 = L.utm(x_, y_, 48, 'N', false);
				ll0 = utm0.latLng();
				if (ll0){
					markerPosition.setLatLng(ll0);
					polyline_.addLatLng(ll0);
				}

				// console.log("path: ", JSON.stringify(path_));

				// If the marker has went out of the map, we move the map
				// bounds = map.getBounds();
				// if(!bounds.contains(ll0))
				// 	map.setView(ll0, zoomLevel);
				
				// console.log("Update position");
			}

			i1++;

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
			messageType : 'nav_msgs/Path'
		});

		var i2 = 0;
		listenerPose.subscribe(function(message2) {

			if(loadedMap2 == false) 
			{
				polyline2_ = L.polyline(path_, {color: 'blue'}).addTo(map);

				loadedMap2 = true;

			} else if(i2 % paramNbCyclesValue == 0) {
				// 0 Declaration of variables
				var latlngs = [];
				var messages = message2.poses;
				var path_length = messages.length;

				for (var i = 0; i < path_length; i++) {
					var scale0 = new THREE.Vector3 (1, 1, 1);
					var rot_body = new THREE.Quaternion( messages[i].pose.orientation.x, 
															messages[i].pose.orientation.y, 
															messages[i].pose.orientation.z, 
															messages[i].pose.orientation.w );

					var trans_body = new THREE.Vector3( messages[i].pose.position.x, 
													 messages[i].pose.position.y, 
													 messages[i].pose.position.z);	
					
					var x2_, y2_;
					var cam2gps0 = new THREE.Matrix4();
					var imu02enu_ = imu02enu.clone();

					if(z_up == true){
						x2_ = trans_body.x + translation0.x;
						y2_ = trans_body.y + translation0.y;
					} else {

						// 1 R(camk2cam0) * R(imuk2camk) = R(imuk2cam0)
						rot_body.multiply(quaternion_imu2cam).normalize();
		
						cam2gps0.compose(trans_body, rot_body, scale0);
		
						// 2 R(imu02enu) * R(cam02imu0) * R(imuk2cam0) = R(imuk2enu)

						imu02enu_.multiply(cam2imu);
		
						imu02enu_.multiply(cam2gps0);
		
						imu02enu_.decompose(trans_body, rot_body, scale0);
		
						x2_ = trans_body.x;
						y2_ = trans_body.y;
					}

					var utm = L.utm(x2_, y2_, 48, 'N', false);
					var ll = utm.latLng();

					latlngs.push(ll);

				}

				// markerPosition2.setLatLng(ll);
				// polyline2_.addLatLng(ll);

				polyline2_.setLatLngs(latlngs);
				markerPosition2.setLatLng(latlngs[path_length-1]);
				// console.log("index of last element ", path_length-1);
				
			}
			i2 ++;
		});
	});
});
