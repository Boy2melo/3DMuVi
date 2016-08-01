#ifndef IPOSEESTIMATOR_H
#define IPOSEESTIMATOR_H

#include "ialgorithm.h"

class CInputDataSet;
class CDataFeature;
class CDataPose;

/*!
 * \brief An interface declaring methods which need to be implemented by a pose estimator plugin.
 * \author Stefan Wolf
 *
 * A pose estimator should compute the poses from which the given images has been made.
 *
 * A pose estimator plugin needs to export the symbol with the name given by IPoseEstimator::symbol.
 * This symbol should point to a factory function which creates a new object of the plugin's class
 * which should be stored in a std::shared_ptr. The factory funtion needs to be of the type
 * IPoseEstimator::Factory.
 */

class IPoseEstimator : public IAlgorithm
{
public:
  /*!
   * \name Members needed by plugin loader
   * \{
   */

  /*!
   * \brief The name of the symbol of the plugin's factory method.
   */
  static constexpr auto symbol = "newPoseEstimator";
  /*!
   * \brief The type of the plugin's factory function.
   */
  typedef std::shared_ptr<IPoseEstimator> Factory();

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
  static constexpr auto name = "Pose estimator";

  /*!
   * \brief Sets the images which can be used by the plugin.
   * \param images The images which should be used to compute the poses.
   */
  virtual void setImages(std::shared_ptr<CInputDataSet> images) = 0;
  /*!
   * \brief Sets the feature matches which can be used by the plugin.
   * \param featureMatches The feature matches which can be used to compute the poses.
   */
  virtual void setFeatureMatches(std::shared_ptr<CDataFeature> featureMatches) = 0;

  /*!
   * \brief Returns the computed poses.
   * \return The computed poses.
   *
   * This function should not be called before IAlgorithm::run has been called.
   */
  virtual std::shared_ptr<CDataPose> getPoses() = 0;

  /*!
   * \}
   */
};

#endif // IPOSEESTIMATOR_H
