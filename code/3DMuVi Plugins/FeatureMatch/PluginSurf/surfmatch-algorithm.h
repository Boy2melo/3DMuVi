#ifndef SURFMATCHALGORITHM
#define SURFMATCHALGORITHM

#include <cstdint>

#include <tuple>
#include <vector>

#include <opencv2/opencv.hpp>

using FeatureTable = std::vector<std::tuple<std::uint64_t /*feature id*/, float /*x*/, float /*y*/, std::uint32_t /*image_id*/>>;
FeatureTable findSurfMatches(std::vector<cv::Mat> imgs, bool upright, int octaves, int intervals, int init_sample, float thres);

#endif // SURFMATCHALGORITHM

