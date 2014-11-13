#ifndef SEARCH_PLAN_H
#define SEARCH_PLAN_H

#include "observation_pose_collection.h"

#include <visualization_msgs/Marker.h>

#include <vector>
#include <fstream>

template <class CELL_GAIN_LOOKUP>
class SearchPlan
{
public:
    size_t last_idx;
    std::vector<double> time;
    std::vector<double> etime;
    std::vector<double> pdone;
    std::vector<size_t> cam_pose_idx;
    std::vector<detection_t> detected_cells;

    SearchPlan(CELL_GAIN_LOOKUP const & cgl, ObservationPoseCollection const & opc)
        : last_idx(0)
        , time(std::vector<double>(1, 0.0))
        , etime(std::vector<double>(1, 0.0))
        , pdone(std::vector<double>(1, 0.0))
        , cam_pose_idx(std::vector<size_t>(1, -1))
        , detected_cells(std::vector<detection_t>(1))
        , cgl(cgl)
        , opc(opc)
    {}

    void appendPose(size_t const next_cam_pose_idx) {
        double duration = opc.getTravelTime(cam_pose_idx[last_idx], next_cam_pose_idx);
        double eduration = (1.0 - pdone[last_idx]) * duration;
        detection_t const & new_detection = opc.getPoses()[next_cam_pose_idx].cell_id_sets[0];
        cam_pose_idx.push_back(next_cam_pose_idx);
        time.push_back(time[last_idx] + duration);
        etime.push_back(etime[last_idx] + eduration);
        pdone.push_back(pdone[last_idx] + calcGain(detected_cells[last_idx], new_detection));
        detected_cells.push_back(detection_union(detected_cells[last_idx], new_detection));
        ++last_idx;
    }

    void greedy(double const horizon) {
        double tmax = time[last_idx] + horizon;
        while(time[last_idx] < tmax) {
            double max_utility = 0.0;
            size_t max_utility_idx = 0;
            for(size_t i = 0; i < opc.getPoses().size(); ++i) {
                if(i == cam_pose_idx[last_idx]) continue;
                double duration = opc.getTravelTime(cam_pose_idx[last_idx], i);
                detection_t const & new_detection = opc.getPoses()[i].cell_id_sets[0];
                double gain = calcGain(detected_cells[last_idx], new_detection);
                double utility = gain / duration;
                if(utility > max_utility) {
                    max_utility = utility;
                    max_utility_idx = i;
                }
            }
            if(max_utility < std::numeric_limits<double>::epsilon()) {
                // Cannot find more useful observation poses
                break;
            } else {
                appendPose(max_utility_idx);
            }
        }
    }

    void sendMarker(std::string frame_id, ros::Publisher & pub)
    {
        for(size_t i = 1; i <= last_idx; ++i) {
            visualization_msgs::Marker marker;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 0.2;
            if(i > 1) {
                marker.color.r = pdone[i] - pdone[i-1];
                marker.color.g = 1.0 - marker.color.r;
            } else {
                marker.color.b = 1.0;
            }
            marker.color.a = 0.5;
            tf::poseTFToMsg(opc.getPoses()[cam_pose_idx[i]].pose, marker.pose);
            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time::now();
            marker.id = i;
            marker.ns = "search_plan";
            pub.publish(marker);
        }
        // Connecting lines
        for(size_t i = 1; i+1 <= last_idx; ++i) {
            visualization_msgs::Marker marker;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.scale.x = 0.02;
            marker.scale.z = std::numeric_limits<float>::epsilon();
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            geometry_msgs::Point p;
            tf::pointTFToMsg(opc.getPoses()[cam_pose_idx[i]].pose.getOrigin(), p);
            marker.points.push_back(p);
            tf::pointTFToMsg(opc.getPoses()[cam_pose_idx[i+1]].pose.getOrigin(), p);
            marker.points.push_back(p);
            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time::now();
            marker.id = last_idx + 1 + i;
            marker.ns = "search_plan";
            pub.publish(marker);
        }
    }

    void writeTimeplot(std::string const & filename)
    {
        std::ofstream f;
        f.open(filename.c_str());
        f << "pDone\ttime\tetime\n";
        for(size_t i = 0; i <= last_idx; ++i) {
            f << pdone[i] << "\t" << time[i] << "\t" << etime[i] << "\n";
        }
        f.close();
    }

private:
    CELL_GAIN_LOOKUP const & cgl;
    ObservationPoseCollection const & opc;

    double calcGain(detection_t const & a, detection_t const & b)
    {
        double gain = 0.0;
        for(detection_t::iterator it = b.begin(); it != b.end(); ++it) {
            if(!a.count(*it)) {
                gain += cgl(*it);
            }
        }
        return gain;
    }

    static detection_t detection_union(detection_t const & a, detection_t const & b)
    {
        if(a.size() >= b.size()) {
            detection_t du;
            du.rehash((a.size() + b.size()) / du.max_load_factor());
            du.insert(a.begin(), a.end());
            du.insert(b.begin(), b.end());
            return du;
        } else {
            return detection_union(b, a);
        }
    }
};

#endif // SEARCH_PLAN_H
