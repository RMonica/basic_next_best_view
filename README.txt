-- INSTALLATION --

This is a ROS package. Compile it with catkin.

-- BASIC USAGE --

Set the parameters:
  voxel_resolution (float): resolution of the voxel grid (edge of each voxel)
  sensor_range_max (float): max range of the sensor
  sphere_radius (float): radius of the view sphere
  horizontal_resolution (uint): horizontal resolution of the sensor (lower to increase speed)
  vertical_resolution (uint): vertical resolution of the sensor (lower to increase speed)
  half_fov_horizontal (float): horizontal HFOV of the sensor
  half_fov_vertical (float): vertical HFOV of the sensor
  latitude_max, latitude_min (float): latitude range when sampling the view sphere (deg)
  longitude_max, longitude_min (float): longitude range when sampling the view sphere (deg)
  zrotation_max, zrotation_min (float): range of the camera rotation around the z-axis when sampling the view sphere (deg)
  latitude_step, longitude_step, zrotation_step (float): steps when sampling the view sphere (deg)
  occupied_voxel_weight, unknown_voxel_weight (float): weight of unknown and occupied voxels in the objective function

Send the current point of interest, i.e. the center of the view sphere, to topic:

  /basic_next_best_view/poi (geometry_msgs/PointStamped)

Send the current occupancy voxel grid to the topic:

  /basic_next_best_view/occupancy (sensor_msgs/PointCloud2)

This message must be obtained from a pcl::PointCloud<pcl::PointXYZI>.
If the intensity I of a point is:
  I < 0: the point is empty
  I > 0: the point is occupied
  I = 0: the point is unknown
Missing points in the voxel grid are considered empty.

Call the Execute action. Fill the basic_request field with:
  type = REQUEST_GET_BEST_POSES
  has_index_range = false

The action will return an array of poses in the field basic_response.best_poses, ordered from the fittest (at index 0), based on the number of unknown and/or known voxels visible from that pose, depending on the metric.

-- ADVANCED USAGE --

TODO

-- CREDITS --

basic_next_best_view by Dario Lodi Rizzini and Riccardo Monica, RIMLab, Department of Information Engineering, University of Parma, Italy.
http://www.rimlab.ce.unipr.it/

Developed for the paper:
Global Registration of Mid-Range 3D Observations and Short Range Next Best Views,
Jacopo Aleotti, Dario Lodi Rizzini, Riccardo Monica and Stefano Caselli,
IEEE/RSJ International Conference on Intelligent Robots and Systems, 2014

Released under the BSD license.

2014-12-02
