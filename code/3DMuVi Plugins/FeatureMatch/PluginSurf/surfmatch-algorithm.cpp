#include <cstdint>

#include <algorithm>
#include <iostream>
#include <map>
#include <tuple>

#include <QImage>

#include <opencv2/opencv.hpp>
#include <surflib.h>

using namespace std;
using namespace cv;

#define log(x) cerr << (x)
#define warn 'Warning: '

using PointCoordinates = pair<float, float>;
PointCoordinates fromIpoint(Ipoint pt)
{
  return {pt.x, pt.y};
}

using FeatureTable = vector<tuple<uint64_t /*feature id*/, float /*x*/, float /*y*/, uint32_t /*image_id*/>>;
FeatureTable findSurfMatches(vector<Mat> imgs, bool upright, int octaves, int intervals, int init_sample, float thres)
{
  IpVec ipts1, ipts2;
  IpVec * pIpts1 = &ipts1;
  IpVec * pIpts2 = &ipts2;

  using FeatureMap = map<PointCoordinates /*feature location*/, int /*feature id*/>;
  vector<FeatureMap> ids(imgs.size());
  vector<FeatureMap>::iterator pfm1 = ids.end();
  vector<FeatureMap>::iterator pfm2 = ids.begin();
  int current_feature = 0;

  for (IplImage img : imgs)
  {
    pIpts1->clear();
    surfDetDes(&img, *pIpts1, upright, octaves, intervals, init_sample, thres);
    swap(pIpts1, pIpts2);

    IpPairVec matches;
    getMatches(*pIpts1, *pIpts2, matches);

    for (auto const & match : matches)
    {
      int feature_id = current_feature;
      auto ret = pfm1->insert({fromIpoint(match.first), feature_id});
      if (ret.second)
      {
        ++current_feature;
      }
      else
      {
        feature_id = ret.first->second;
      }

      pfm2->insert({fromIpoint(match.second), feature_id}); // TODO: check for duplicates
    }

    pfm1 = pfm2++;
  }

  FeatureTable features;
  for (size_t img = 0; img != ids.size(); ++img)
  {
    for (auto const & feature : ids[img])
    {
      features.push_back(make_tuple(feature.second, feature.first.first, feature.first.second, img));
    }
  }

  return features;
}
