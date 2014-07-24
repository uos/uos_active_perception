#include "next_best_view_node.h"

#include "active_perception_map.h"

#include <ros/ros.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <visualization_msgs/Marker.h>
#include <boost/numeric/interval.hpp>
#include <boost/random.hpp>

#include <ctime>

NextBestViewNode::NextBestViewNode() :
    m_node_handle("~"),
    m_node_handle_pub(),
    m_point_cloud_subscriber(m_node_handle_pub.subscribe(
                                 "cloud_in",
                                 1,
                                 &NextBestViewNode::pointCloudCb,
                                 this)),
    m_get_bbox_percent_unseen_server(m_node_handle_pub.advertiseService(
                                         "/get_bbox_occupancy",
                                         &NextBestViewNode::getBboxOccupancyCb,
                                         this)),
    m_get_observation_camera_poses_server(m_node_handle_pub.advertiseService(
                                              "/get_observation_camera_poses",
                                              &NextBestViewNode::getObservationCameraPosesCb,
                                              this)),
    m_get_objects_to_remove_server(m_node_handle_pub.advertiseService(
                                       "/get_objects_to_remove",
                                       &NextBestViewNode::getObjectsToRemoveCb,
                                       this)),
    m_tf_listener(),
    m_marker_pub(m_node_handle_pub.advertise<visualization_msgs::Marker>("/next_best_view_marker", 10000)),
    m_perception_map(0.01)
{
    m_node_handle.param("resolution"    , m_resolution    , 0.05);
    m_node_handle.param("ray_skip"      , m_ray_skip      , 1.00);
    m_node_handle.param("world_frame_id", m_world_frame_id, std::string("/odom_combined"));

    // Set camera constraints from parameters (Defaults: xtion on calvin)
    m_node_handle.param("camera_frame_id",m_camera_constraints.frame_id  , std::string("/head_mount_kinect"));
    m_node_handle.param("height_min"    , m_camera_constraints.height_min, 1.5875);
    m_node_handle.param("height_max"    , m_camera_constraints.height_max, 1.5875);
    m_node_handle.param("pitch_min"     , m_camera_constraints.pitch_min , -0.935815); // [-53.6182 deg]
    m_node_handle.param("pitch_max"     , m_camera_constraints.pitch_max , -0.935815); // [-53.6182 deg]
    m_node_handle.param("range_min"     , m_camera_constraints.range_min , 0.4);
    m_node_handle.param("range_max"     , m_camera_constraints.range_max , 3.0);
    m_node_handle.param("hfov"          , m_camera_constraints.hfov      , 1.01229097);
    m_node_handle.param("vfov"          , m_camera_constraints.vfov      , 0.785398163);
    m_node_handle.param("roll"          , m_camera_constraints.roll      , PI);

    m_perception_map.setResolution(m_resolution);
}

double NextBestViewNode::getIntersectionVolume(
        const octomath::Vector3 &box1min,
        const octomath::Vector3 &box1max,
        const octomath::Vector3 &box2min,
        const octomath::Vector3 &box2max
){
    octomath::Vector3 max, min;
    // intersection min is the maximum of the two boxes' mins
    // intersection max is the minimum of the two boxes' maxes
    max.x() = std::min(box1max.x(), box2max.x());
    max.y() = std::min(box1max.y(), box2max.y());
    max.z() = std::min(box1max.z(), box2max.z());
    min.x() = std::max(box1min.x(), box2min.x());
    min.y() = std::max(box1min.y(), box2min.y());
    min.z() = std::max(box1min.z(), box2min.z());
    // make sure that max is actually larger in each dimension
    if(max.x() < min.x()) return 0.0;
    if(max.y() < min.y()) return 0.0;
    if(max.z() < min.z()) return 0.0;
    return (max.x() - min.x()) * (max.y() - min.y()) * (max.z() - min.z());
}

std::vector<tf::Vector3> NextBestViewNode::bboxVertices(race_msgs::BoundingBox const & bbox)
{
    // Construct a vector of bounding box vertices relative to the bounding box pose
    std::vector<tf::Vector3> bbox_points(8);
    bbox_points[0] = tf::Vector3(-bbox.dimensions.x/2, -bbox.dimensions.y/2, +bbox.dimensions.z/2);
    bbox_points[1] = tf::Vector3(-bbox.dimensions.x/2, -bbox.dimensions.y/2, -bbox.dimensions.z/2);
    bbox_points[2] = tf::Vector3(-bbox.dimensions.x/2, +bbox.dimensions.y/2, +bbox.dimensions.z/2);
    bbox_points[3] = tf::Vector3(-bbox.dimensions.x/2, +bbox.dimensions.y/2, -bbox.dimensions.z/2);
    bbox_points[4] = tf::Vector3(+bbox.dimensions.x/2, -bbox.dimensions.y/2, +bbox.dimensions.z/2);
    bbox_points[5] = tf::Vector3(+bbox.dimensions.x/2, -bbox.dimensions.y/2, -bbox.dimensions.z/2);
    bbox_points[6] = tf::Vector3(+bbox.dimensions.x/2, +bbox.dimensions.y/2, +bbox.dimensions.z/2);
    bbox_points[7] = tf::Vector3(+bbox.dimensions.x/2, +bbox.dimensions.y/2, -bbox.dimensions.z/2);
    // Transform points to bbox frame
    tf::Quaternion tf_orientation;
    tf::Point tf_position;
    tf::quaternionMsgToTF(bbox.pose_stamped.pose.orientation, tf_orientation);
    tf::pointMsgToTF(bbox.pose_stamped.pose.position, tf_position);
    tf::Transform transform(tf_orientation, tf_position);
    for(std::vector<tf::Vector3>::iterator it = bbox_points.begin(); it != bbox_points.end(); ++it)
    {
        *it = transform(*it);
    }
    return bbox_points;
}

bool NextBestViewNode::getAxisAlignedBounds(
        race_msgs::BoundingBox const & bbox,
        octomath::Vector3 & min,
        octomath::Vector3 & max
) const
{
    float inf = std::numeric_limits<float>::infinity();
    min.x() =  inf; min.y() =  inf; min.z() =  inf;
    max.x() = -inf; max.y() = -inf; max.z() = -inf;

    std::vector<tf::Vector3> bbox_vertices = bboxVertices(bbox);
    if(!m_tf_listener.waitForTransform(m_world_frame_id,
                                       bbox.pose_stamped.header.frame_id,
                                       bbox.pose_stamped.header.stamp,
                                       ros::Duration(1)))
    {
        ROS_ERROR_STREAM("next_best_view_node: Timed out while waiting for transform from " <<
                          bbox.pose_stamped.header.frame_id << " to " <<
                          m_world_frame_id);
        return false;
    }
    for(std::vector<tf::Vector3>::iterator it = bbox_vertices.begin(); it != bbox_vertices.end(); ++it)
    {
        geometry_msgs::PointStamped pin, pout;
        pin.header = bbox.pose_stamped.header;
        pin.point.x = it->x();
        pin.point.y = it->y();
        pin.point.z = it->z();
        m_tf_listener.transformPoint(m_world_frame_id, pin, pout);
        if(pout.point.x < min.x()) min.x() = pout.point.x;
        if(pout.point.y < min.y()) min.y() = pout.point.y;
        if(pout.point.z < min.z()) min.z() = pout.point.z;
        if(pout.point.x > max.x()) max.x() = pout.point.x;
        if(pout.point.y > max.y()) max.y() = pout.point.y;
        if(pout.point.z > max.z()) max.z() = pout.point.z;
    }
    return true;
}

/**
  ASSUMPTION: Camera constraints are defined in the world frame.
  */
std::vector<tf::Transform> NextBestViewNode::sampleObservationSpace
(
        std::vector<octomap::point3d> const & points_of_interest,
        int sample_size
) const
{
    boost::mt19937 rng(std::time(0));
    std::vector<tf::Transform> samples;
    for(int i = 0; i < sample_size; ++i)
    {
        // Pick a random poi as center
        octomap::point3d const & poi = points_of_interest.at(
                    boost::uniform_int<>(0, points_of_interest.size() - 1)(rng));

        // Find parameter constraints
        using boost::numeric::intersect;
        typedef boost::numeric::interval<
                    double,
                    boost::numeric::interval_lib::policies<
                        boost::numeric::interval_lib::save_state<
                            boost::numeric::interval_lib::rounded_transc_std<double> >,
                        boost::numeric::interval_lib::checking_base<double> > >
                Interval;
        // Hardware constraints
        Interval r(m_camera_constraints.range_min, m_camera_constraints.range_max);
        Interval dH(poi.z() - m_camera_constraints.height_max, poi.z() - m_camera_constraints.height_min);
        Interval p(m_camera_constraints.pitch_min - m_camera_constraints.vfov / 2.0,
                   m_camera_constraints.pitch_max + m_camera_constraints.vfov / 2.0);
        Interval sin_p = boost::numeric::sin(p);
        // Possible view angles according to hardware constraints
        sin_p = intersect(sin_p, dH / r);

        if(boost::numeric::empty(sin_p))
        {
            ROS_WARN_STREAM("Skipped unobservable POI: z=" << poi.z());
            continue;
        }

        // Select a distance and height (randomize selection order)
        double distance, dHeight;
        if(boost::uniform_01<>()(rng) < 0.5)
        {
            // possible ranges that satisfy view distance, height and pitch constraints
            r = intersect(r, dH / sin_p);
            distance = r.lower() + boost::uniform_01<>()(rng) * (r.upper() - r.lower());
            dH = intersect(dH, distance * sin_p);
            dHeight = dH.lower() + boost::uniform_01<>()(rng) * (dH.upper() - dH.lower());
        }
        else
        {
            // possible height offsets that satisfy view distance, height and pitch constraints
            dH = intersect(dH, r * sin_p);
            dHeight = dH.lower() + boost::uniform_01<>()(rng) * (dH.upper() - dH.lower());
            r = intersect(r, dHeight / sin_p);
            distance = r.lower() + boost::uniform_01<>()(rng) * (r.upper() - r.lower());
        }

        // Create random position within feasible range and height
        double direction = boost::uniform_01<>()(rng) * 2.0 * PI;
        double x_offset = distance * std::cos(direction);
        double y_offset = distance * std::sin(direction);
        tf::Vector3 position(poi.x() + x_offset,
                             poi.y() + y_offset,
                             poi.z() - dHeight);

        // Adjust camera pitch
        double pitch = std::asin(dHeight / distance);
        if(pitch > m_camera_constraints.pitch_max)
        {
            pitch = m_camera_constraints.pitch_max;
        }
        else if(pitch < m_camera_constraints.pitch_min)
        {
            pitch = m_camera_constraints.pitch_min;
        }

        // Point the created pose towards the target voxel
        // We want to do intrinsic YPR which is equivalent to fixed axis RPY
        tf::Quaternion orientation;
        orientation.setRPY(m_camera_constraints.roll, -pitch, PI + direction);

        samples.push_back(tf::Transform(orientation, position));
    }
    return samples;
}

bool NextBestViewNode::getBboxOccupancyCb(race_next_best_view::GetBboxOccupancy::Request &req,
                                          race_next_best_view::GetBboxOccupancy::Response &res)
{
    octomath::Vector3 min, max;
    getAxisAlignedBounds(req.bbox, min, max);
    double total_volume = (max.x() - min.x()) * (max.y() - min.y()) * (max.z() - min.z());
    double free_volume = 0.0, occupied_volume = 0.0;
    for(octomap::OcTree::leaf_bbx_iterator it = m_perception_map.getOccupancyMap().begin_leafs_bbx(min,max),
        end = m_perception_map.getOccupancyMap().end_leafs_bbx();
        it != end; ++it)
    {
        double side_len_half = it.getSize() / 2.0;
        octomath::Vector3 vox_min = it.getCoordinate(), vox_max = it.getCoordinate();
        vox_min.x() -= side_len_half;
        vox_min.y() -= side_len_half;
        vox_min.z() -= side_len_half;
        vox_max.x() += side_len_half;
        vox_max.y() += side_len_half;
        vox_max.z() += side_len_half;
        double v = getIntersectionVolume(min, max, vox_min, vox_max);
        if(m_perception_map.getOccupancyMap().isNodeOccupied(*it))
        {
            occupied_volume += v;
        }
        else
        {
            free_volume += v;
        }
    }
    res.free = free_volume / total_volume;
    res.occupied = occupied_volume / total_volume;
    res.unknown = 1.0 - res.free - res.occupied;
    return true;
}

bool NextBestViewNode::getObservationCameraPosesCb(race_next_best_view::GetObservationCameraPoses::Request& req,
                                                   race_next_best_view::GetObservationCameraPoses::Response& res)
{
    boost::mt19937 rng(std::time(0));
    boost::uniform_01<> rand_u01;
    // TODO: Shrink the ROI to an area that is in principle observable
    // (not higher/lower than the camera constraints allow observations to be made)
    // Should be implemented together with the ObservationSampler class which will wrap the
    // sampleObservationSpace method and allow to ask for single samples

    std::vector<visualization_msgs::Marker> markers;
    double max_gain = m_resolution;

    // Some derived camera parameters
    double azimuth_min = -m_camera_constraints.hfov / 2.0;
    double azimuth_max =  m_camera_constraints.hfov / 2.0;
    double inclination_min = -m_camera_constraints.vfov / 2.0;
    double inclination_max =  m_camera_constraints.vfov / 2.0;
    // Find the right discretization of ray angles so that each octree voxel at max range is hit by one ray.
    double angle_increment =
            std::acos(1 - (std::pow(m_resolution, 2) / (2.0 * std::pow(m_camera_constraints.range_max, 2))));

    // Gather unknown voxel centers
    // TODO: This whole method of copying voxel centers into a vector is rather costly for many fringe voxels.
    //       The sampling procedure could be reformulated in such a way that we only need to iterate once through
    //       all the fringe voxels in the octree.
    std::vector<octomap::point3d> fringe_centers;
    if(req.roi.empty())
    {
        // If the request frame id is empty, use all fringe voxels
        fringe_centers = m_perception_map.getFringeCenters();
    }
    else
    {
        octomath::Vector3 roi_min, roi_max;
        // TODO: Evaluate the other roi elements
        if(!getAxisAlignedBounds(req.roi[0], roi_min, roi_max))
        {
            return false;
        }
        fringe_centers = m_perception_map.getFringeCenters(roi_min, roi_max);
        // TODO: What happens if there are no fringe voxels in the ROI (because it is inside unknown space)?
    }

    std::vector<tf::Transform> samples = sampleObservationSpace(fringe_centers, req.sample_size);
    for(std::vector<tf::Transform>::iterator pose_it = samples.begin(); pose_it != samples.end(); ++pose_it)
    {
        octomap::point3d cam_point = octomap::pointTfToOctomap(pose_it->getOrigin());

        // Send a marker
        visualization_msgs::Marker lines_marker;
        lines_marker.action = visualization_msgs::Marker::ADD;
        lines_marker.type = visualization_msgs::Marker::LINE_LIST;
        lines_marker.lifetime = ros::Duration();
        lines_marker.scale.x = 0.001;
        lines_marker.scale.y = 0.001;
        lines_marker.scale.z = 0.001;
        lines_marker.color.g = 1.0;
        lines_marker.color.r = 1.0;
        lines_marker.color.b = 1.0;
        lines_marker.color.a = 0.5;
        lines_marker.header.frame_id = m_world_frame_id;
        lines_marker.header.stamp = ros::Time::now();
        static int markerid = 0;
        lines_marker.id = markerid++;
        lines_marker.ns = "nbv_viewlines";

        double gain = 0.0;
        int n_rays = 0;
        for(double azimuth = azimuth_min; azimuth <= azimuth_max; azimuth += angle_increment)
        {
            for(double inclination = inclination_min; inclination <= inclination_max; inclination += angle_increment)
            {
                if(rand_u01(rng) < req.ray_skip)
                {
                    continue;
                }
                n_rays++;
                tf::Vector3 ray_end_in_cam(m_camera_constraints.range_max * std::cos(azimuth),
                                           m_camera_constraints.range_max * std::sin(azimuth),
                                           m_camera_constraints.range_max * std::sin(inclination));
                octomap::point3d ray_end = octomap::pointTfToOctomap((*pose_it)(ray_end_in_cam));
                double gaingain = m_perception_map.estimateRayGain(cam_point, ray_end);
                gain += gaingain;
                if(gaingain > m_resolution) {
                    lines_marker.points.push_back(octomap::pointOctomapToMsg(cam_point));
                    lines_marker.points.push_back(octomap::pointOctomapToMsg(ray_end));
                }
            }
        }
        gain /= n_rays;
        // Write pose candidate to answer
        geometry_msgs::Pose camera_pose_msg;
        tf::poseTFToMsg(*pose_it, camera_pose_msg);
        if(gain > max_gain)
        {
            max_gain = gain;
        }
        if(gain > m_resolution)
        {
            res.camera_poses.push_back(camera_pose_msg);
            res.information_gain.push_back(gain);
        }

        // Send a marker
        //markers.push_back(lines_marker);
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.lifetime = ros::Duration(10);
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 0.2;
        if(gain > m_resolution) {
            marker.color.g = gain;
        } else {
            marker.color.b = 1.0;
        }
        marker.color.a = 0.5;
        marker.pose = camera_pose_msg;
        marker.header.frame_id = m_world_frame_id;
        marker.header.stamp = ros::Time::now();
        marker.id = markerid++;
        marker.ns = "nbv_samples";
        markers.push_back(marker);
    }

    // Publish markers
    ROS_INFO_STREAM("Publishing " << markers.size() << " markers");
    for(std::vector<visualization_msgs::Marker>::iterator it = markers.begin(); it != markers.end(); ++it)
    {
        if(it->color.b < 1.0) {
            it->color.g /= max_gain;
            it->color.r = 1.0 - it->color.g;
        }
        m_marker_pub.publish(*it);
    }

    return true;
}

bool NextBestViewNode::getObjectsToRemoveCb(race_next_best_view::GetObjectsToRemove::Request& req,
                                            race_next_best_view::GetObjectsToRemove::Response& res)
{
    // TODO: Implement
    return false;
}

void NextBestViewNode::pointCloudCb(sensor_msgs::PointCloud2 const & cloud)
{
    // Find sensor origin
    tf::StampedTransform sensor_to_world_tf, camera_to_world_tf;
    try
    {
        m_tf_listener.waitForTransform(m_world_frame_id, cloud.header.frame_id, cloud.header.stamp, ros::Duration(2.0));
        m_tf_listener.lookupTransform(m_world_frame_id, cloud.header.frame_id, cloud.header.stamp, sensor_to_world_tf);
        m_tf_listener.lookupTransform(m_world_frame_id, m_camera_constraints.frame_id, cloud.header.stamp, camera_to_world_tf);
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR_STREAM("next_best_view_node: Transform error of sensor data: "
                         << ex.what()
                         << ", cannot integrate data.");
        return;
    }

    // Ugly converter cascade to get octomap_cloud
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    octomap::Pointcloud octomap_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);
    octomap::pointcloudPCLToOctomap(pcl_cloud, octomap_cloud);

    // Integrate point cloud
    m_perception_map.integratePointCloud(octomap_cloud, octomap::poseTfToOctomap(sensor_to_world_tf), camera_to_world_tf, m_camera_constraints);

    // Publish rviz map visualization
    visualization_msgs::Marker marker = m_perception_map.genOccupancyMarker();
    marker.header.frame_id = m_world_frame_id;
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.ns = "occupancy_map";
    m_marker_pub.publish(marker);
    marker = m_perception_map.genFringeMarker();
    marker.header.frame_id = m_world_frame_id;
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.ns = "fringe_map";
    m_marker_pub.publish(marker);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "next_best_view_node");
    NextBestViewNode node;
    ROS_INFO("next_best_view_node: Initialized!");
    ros::spin();
}
