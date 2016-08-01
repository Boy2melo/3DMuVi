#ifndef IFEATUREMATCHER_H
#define IFEATUREMATCHER_H

#include "ialgorithm.h"

class CInputDataSet;
class CDataFeature;

/*!
 * \brief An interface declaring methods which need to be implemented by a feature matcher plugin.
 * \author Stefan Wolf
 *
 * A feature matcher should find feature points on the given images. Moreover, it should match those
 * features across the images.

 * A feature matcher plugin needs to export the symbol with the name given by
 * IFeatureMatcher::symbol. This symbol should point to a function which creates a new object of the
 * plugin's class which should be stored in a std::shared_ptr. The funtion needs to be of the type
 * IFeatureMatcher::Factory.
 */
class IFeatureMatcher : public IAlgorithm
{
public:
  /*!
   * \name Members needed by plugin loader
   * \{
   */

  /*!
   * \brief The name of the symbol of the plugin's factory method.
   */
  static constexpr auto symbol = "newFeatureMatcher";
  /*!
   * \brief The type of the plugin's factory function.
   */
  typedef std::shared_ptr<IFeatureMatcher> Factory();

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
  static constexpr auto name = "Feature matcher";

  /*!
   * \brief Sets the images which can be used by the plugin.
   * \param images The images which should be used to compute the features.
   */
  virtual void setImages(std::shared_ptr<CInputDataSet> images) = 0;

  /*!
   * \brief Returns the found feature matches.
   * \return The found feature matches.
   *
   * This function should not be called before IAlgorithm::run has been called.
   */
  virtual std::shared_ptr<CDataFeature> getFeatureMatches() = 0;

  /*!
   * \}
   */
};

#endif // IFEATUREMATCHER_H
