#include "kmeans.h"

#include <boost/random.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>

Kmeans::Kmeans(size_t const k, std::vector<Point> points)
    : points(points)
    , centroid_idxs(k)
    , assignment(points.size())
{
    boost::mt19937 rng;
    boost::uniform_01<> rand_u01;

    // Randomly initialize centroids
    for(size_t i = 0; i < k; ++i) {
        centroid_idxs[i] = (size_t) (rand_u01(rng) * points.size());
    }

    converge();
}

void Kmeans::addCluster()
{
    centroid_idxs.push_back(max_remote_point_idx);
    converge();
}

void Kmeans::converge()
{
    // Assign clusters and recalculate centroids until convergence
    do {
        max_remote_point_idx = assign(points, centroid_idxs, assignment);
    } while(find_centroids(points, assignment, centroid_idxs));
    max_remote_distance = std::sqrt(dist2(points[max_remote_point_idx], points[centroid_idxs[assignment[max_remote_point_idx]]]));
}

const Kmeans::Point & Kmeans::getCentroid(const size_t point_id) const
{
    return points[centroid_idxs[assignment[point_id]]];
}

std::vector<Kmeans::Point> const & Kmeans::getPoints() const
{
    return points;
}

std::vector<size_t> const & Kmeans::getCentroidIdxs() const
{
    return centroid_idxs;
}

std::vector<size_t> const & Kmeans::getAssignment() const
{
    return assignment;
}

double Kmeans::getMaxRemoteDistance() const
{
    return max_remote_distance;
}

size_t Kmeans::getK() const
{
    return centroid_idxs.size();
}

void Kmeans::writeTabFiles() const
{
    std::ofstream f;
    f.open ("k-means-pts.tab");
    f << "x\ty\tcluster\n";
    f << "c\tc\td\n";
    f << "\t\tc\n";
    for(size_t i = 0; i < points.size(); ++i) {
        f << points[i].x << "\t" << points[i].y << "\t" << assignment[i] << "\n";
    }
    f.close();
    f.open ("k-means-centers.tab");
    f << "x\ty\tcluster\n";
    f << "c\tc\td\n";
    f << "\t\tc\n";
    for(size_t i = 0; i < centroid_idxs.size(); ++i) {
        f << points[centroid_idxs[i]].x << "\t" << points[centroid_idxs[i]].y << "\t" << i << "\n";
    }
    f.close();
}

double Kmeans::dist2(const Point & p1, const Point & p2)
{
    return std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2);
}

size_t Kmeans::assign
(
        std::vector<Point> const & points,
        std::vector<size_t> const & centroid_idxs,
        std::vector<size_t> & assignment
){
    double mrd = 0.0;
    size_t mrp_idx = 0;
    for(size_t pidx=0; pidx < points.size(); ++pidx) {
        double mindist = std::numeric_limits<double>::infinity();
        for(size_t cidx=0; cidx < centroid_idxs.size(); ++cidx) {
            double d = dist2(points[pidx], points[centroid_idxs[cidx]]);
            if(d < mindist) {
                mindist = d;
                assignment[pidx] = cidx;
            }
        }
        if(mindist > mrd) {
            mrd = mindist;
            mrp_idx = pidx;
        }
    }
    return mrp_idx;
}

bool Kmeans::find_centroids
(
        std::vector<Point> const & points,
        std::vector<size_t> const & assignment,
        std::vector<size_t> & centroid_idxs
){
    std::vector<double> xsum(centroid_idxs.size(), 0.0);
    std::vector<double> ysum(centroid_idxs.size(), 0.0);
    std::vector<size_t> csize(centroid_idxs.size(), 0);
    bool change = false;

    // Make sums and counts for all clusters
    for(size_t pidx=0; pidx < points.size(); ++pidx) {
        xsum[assignment[pidx]] += points[pidx].x;
        ysum[assignment[pidx]] += points[pidx].y;
        csize[assignment[pidx]]++;
    }

    // Assign nearest-to-centroid point in cluster as centroid
    for(size_t cidx=0; cidx < centroid_idxs.size(); ++cidx) {
        Point c;
        c.x = xsum[cidx] / csize[cidx];
        c.y = ysum[cidx] / csize[cidx];
        double mindist = std::numeric_limits<double>::infinity();
        size_t old_centroid = centroid_idxs[cidx];
        for(size_t pidx=0; pidx < points.size(); ++pidx) {
            double d = dist2(c, points[pidx]);
            if(d < mindist) {
                mindist = d;
                centroid_idxs[cidx] = pidx;
            }
        }
        if(old_centroid != centroid_idxs[cidx]) {
            change = true;
        }
    }

    return change;
}
