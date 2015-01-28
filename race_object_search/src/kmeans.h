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

    /**
      Initialize and run k-means clustering with k clusters on points.
      The clustering result is available via members of the Kmeans object returned.
      */
    Kmeans(size_t const k, std::vector<Point> points);

    /**
      Increment k and re-run clustering. The additional centroid is initialized at the position of the point with the
      highest assignemt distance in the current clustering.
      */
    void addCluster();

    /**
      Returns the centroid point assigned to the given point_id
      */
    const Point & getCentroid(const size_t point_id) const;

    /**
      Returns the vector of all points. Retrieve a point for a given point_id with getPoints()[point_id].
      */
    std::vector<Point> const & getPoints() const;

    /**
      The returned vector is a mapping from cluster_id to the point_id of the centroid point:
      getCentroidIdxs()[cluster_id] = point_id
      */
    std::vector<size_t> const & getCentroidIdxs() const;

    /**
      The returned vector is a mapping from point_id to the cluster_id of the assigned cluster for that point.
      getAssignment()[point_id] = cluster_id
      */
    std::vector<size_t> const & getAssignment() const;

    /**
      Returns the highest distance from any point to its assigned centroid.
      */
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
