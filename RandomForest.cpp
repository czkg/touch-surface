#include "RandomForest.h"

#include "Util.h"

#include <proto/rdf.pb.h>
#include <rdf/forest.hpp>

#include <google/protobuf/text_format.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#define COMPACT_GOOGLE_LOG_DEBUG
#include <glog/logging.h>

#include <fcntl.h>
#include <iostream>

#include <boost/filesystem.hpp>

using namespace std;
using namespace cv;
using google::protobuf::Message;
using google::protobuf::io::FileInputStream;
namespace fs = boost::filesystem;

using namespace TouchDetectionUtils;

namespace RDFUtils {

    bool readProtoFromText(const std::string& filename, Message* proto) {
        int fd = open(filename.c_str(), O_RDONLY);
        if (fd == -1) {
            LOG(ERROR) << "File not found: " << filename;
            return false;
        }
        FileInputStream* input = new FileInputStream(fd);
        bool success = google::protobuf::TextFormat::Parse(input, proto);
        delete input;
        close(fd);
        return success;
    }

} /* RDFUtils */

using namespace RDFUtils;

RandomForest::RandomForest():m_initialized(false) {
}

RandomForest::~RandomForest() {
}

bool RandomForest::init(const string& rdfProtoFilename,
                        const string& rdfForestDataFile,
                        const cv::Size imageSize) {

    rdf::RDFParameter rdfParam;

    if (!readProtoFromText(rdfProtoFilename, &rdfParam)) {
        std::cerr << "Cannot read proto text!" << '\n';
        m_initialized = false;
        return false;
    }

    const int numTrees   = rdfParam.num_trees();
    const int numLabels  = rdfParam.num_pixels_size();
    const int maxDepth   = rdfParam.num_depth();
    const float min_prob = rdfParam.min_prob();

    // ===== Load trained forest =====
    rdf::Forest forest_(numTrees, maxDepth);
    LOG(INFO) << "Load RDF forest from: " << rdfForestDataFile << '\n';
    std::vector<std::vector<Node_CU> > forest_CU;
    forest_CU.resize(numTrees);
    forest_.readForest(fs::path(rdfForestDataFile), numLabels + 1, forest_CU);

    // ===== GPU Inference =====
    m_cu_infer = std::shared_ptr<CUDA_INFER>(new CUDA_INFER(1,
                                                         imageSize.width,
                                                         imageSize.height,
                                                         true,
                                                         forest_CU));

    m_cu_infer->cu_upload_TreeInfo(numLabels, maxDepth, min_prob, forest_CU);

    m_initialized = true;
}

Mat_<Vec3i> RandomForest::process(const Mat_<float>& depth) {
    if (not m_initialized) {
        LOG(WARNING) << "RDF not initialized" << '\n';
        return Mat_<Vec3i>();
    }

    rdf::DepthImage depthImage;
    depthImage.setDepth(depth / 1000.0);

    Time start = now();

    // Call CUDA functions to do inference
    Mat_<Vec3i> out = m_cu_infer->cu_inferFrame(depthImage.getDepth()).clone();

    LOG_EVERY_N(INFO, 30) << "RDF elapsed time: " << untilNowMs(start) << "ms";

    //result (labels) stored on RGB buffer
    return out;
}
