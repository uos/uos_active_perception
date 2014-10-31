#include "kmeans.h"

#include <boost/random.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <ctime>

int main()
{
    size_t n=10000;
    size_t k=1;
    double max_remote = 0.1;

    std::cout << "Testing k-means with " << n << " random points, k=" << k << ", mrd_target=" << max_remote << std::endl;
    boost::mt19937 rng(std::time(0));
    boost::uniform_01<> rand_u01;
    std::vector<Kmeans::Point> pts(n);
    for(size_t pidx=0; pidx < pts.size(); ++pidx) {
        pts[pidx].x = rand_u01(rng);
        pts[pidx].y = rand_u01(rng);
    }
    std::cout << "Executing k-means ";
    boost::posix_time::ptime tick = boost::posix_time::microsec_clock::local_time();
    Kmeans km(k, pts);
    while(km.getMaxRemoteDistance() > max_remote) {
        km.addCluster();
    }
    boost::posix_time::ptime now  = boost::posix_time::microsec_clock::local_time();
    std::cout << "took msec: " << (now-tick).total_milliseconds() << std::endl;

    std::cout << "final clustering: k=" << km.getK() << ", mrd=" << km.getMaxRemoteDistance() << std::endl;

    km.writeTabFiles();

    return 0;
}
