#ifndef IDEPTHESTIMATOR_H
#define IDEPTHESTIMATOR_H

#include "ialgorithm.h"

class CInputDataSet;
class CDataPose;
class CDataDepth;

/*!
 * \brief An interface declaring methods which need to be implemented by a depth estimator plugin.
 * \author Stefan Wolf
 *
 * A depth estimator should compute depth maps to the given images.
 *
 * A depth estimator plugin needs to export the symbol with the name given by
 * IDepthEstimator::symbol. This symbol should point to a factory function which creates a new
 * object of the plugin's class which should be stored in a std::shared_ptr. The factory funtion
 * needs to be of the type IDepthEstimator::Factory.
 */
class IDepthEstimator : public IAlgorithm
{
public:

  /*!
   * \name Members needed by plugin loader
   * \{
   */

  /*!
   * \brief The name of the symbol of the plugin's factory method.
   */
  static constexpr auto symbol = "newDepthEstimator";
  /*!
   * \brief The type of the plugin's factory function.
   */
  typedef std::shared_ptr<IDepthEstimator> Factory();

  /*!
   * \}
   */

  /*!
   * \name Members needed by workflow
   * \{
   */

  /*!
   * \brief The name of this plugin type.
   */
  static constexpr auto name = "Depth estimator";

  /*!
   * \brief Sets the images which can be used by the plugin.
   * \param images The images which should be used to compute the depth maps.
   */
  virtual void setImages(std::shared_ptr<CInputDataSet> images) = 0;
  /*!
   * \brief Sets the poses which can be used by the plugin.
   * \param poses The poses from which the images have been made.
   */
  virtual void setPoses(std::shared_ptr<CDataPose> poses) = 0;

  /*!
   * \brief Returns the computed depth maps.
   * \return The computed depth maps.
   *
   * This function should not be called before IAlgorithm::run has been called.
   */
  virtual std::shared_ptr<CDataDepth> getDepthMaps() = 0;

  /*!
   * \}
   */
};

#endif // IDEPTHESTIMATOR_H
