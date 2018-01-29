#ifndef _RANDOMFOREST_H
#define _RANDOMFOREST_H

#include <rdf/RDF_CU.cuh>

#include <opencv2/core.hpp>

class RandomForest {
private:
    bool m_forestInSharedMem;
    bool m_initialized;

    std::shared_ptr<CUDA_INFER> m_cu_infer;

public:
    RandomForest ();
    virtual ~RandomForest ();

    bool init(const std::string& rdfProtoFilename, const std::string& rdfForestDataFile, const cv::Size imageSize);
    cv::Mat_<cv::Vec3i> process(const cv::Mat_<float>& depth);
};

#endif /* end of include guard: _RANDOMFOREST_H */
