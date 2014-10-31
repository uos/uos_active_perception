#ifndef KMEANS_H
#define KMEANS_H

#include <vector>
#include <cstddef>

class Kmeans
{
public:
    struct Point
    {
        float x, y;
    };

    Kmeans(size_t const k, std::vector<Point> points);
    void addCluster();
    std::vector<Point> const & getPoints() const;
    std::vector<size_t> const & getCentroidIdxs() const;
    std::vector<size_t> const & getAssignment() const;
    double getMaxRemoteDistance() const;
    size_t getK() const;
    void writeTabFiles() const;

private:
    std::vector<Point> points;
    std::vector<size_t> centroid_idxs;
    std::vector<size_t> assignment;
    double max_remote_distance;
    size_t max_remote_point_idx;

    void converge();
    static double dist2(Point const & p1, Point const & p2);
    static size_t assign(std::vector<Point> const & points,
                         std::vector<size_t> const & centroid_idxs,
                         std::vector<size_t> & assignment);
    static bool find_centroids(std::vector<Point> const & points,
                               std::vector<size_t> const & assignment,
                               std::vector<size_t> & centroid_idxs);
};

#endif // KMEANS_H
