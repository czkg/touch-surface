#ifndef _ORTOOLSINTERFACE_H
#define _ORTOOLSINTERFACE_H

#include <vector>
#include <opencv2/core.hpp>

class DLibInterface {
private:
    DLibInterface ();
    virtual ~DLibInterface ();

public:
    static std::vector<long> linearMatching(const cv::Mat_<int>& matrice);
};

#endif /* end of include guard: _ORTOOLSINTERFACE_H */
