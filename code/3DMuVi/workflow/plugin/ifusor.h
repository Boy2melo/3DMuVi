#ifndef IFUSOR_H
#define IFUSOR_H

#include "ialgorithm.h"

class CInputDataSet;
class CDataDepth;
class CDataFusion;

/*!
 * \brief An interface declaring methods which need to be implemented by a 3D fusor plugin.
 * \author Stefan Wolf
 *
 * A fusor should compute a 3D reconstruction to the given images.
 *
 * A fusor plugin needs to export the symbol with the name given by IFusor::symbol.
 * This symbol should point to a factory function which creates a new object of the plugin's class
 * which should be stored in a std::shared_ptr. The factory funtion needs to be of the type
 * IFusor::Factory.
 */

class IFusor : public IAlgorithm
{
public:
  /*!
   * \name Members needed by plugin loader
   * \{
   */

  /*!
   * \brief The name of the symbol of the plugin's factory method.
   */
  static constexpr auto symbol = "newFusor";
  /*!
   * \brief The type of the plugin's factory function.
   */
  typedef std::shared_ptr<IFusor> Factory();

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
  static constexpr auto name = "3D fusor";

  /*!
   * \brief Sets the images which can be used by the plugin.
   * \param images The images which should be used to compute the 3D reconstruction.
   */
  virtual void setImages(std::shared_ptr<CInputDataSet> images) = 0;
  /*!
   * \brief Sets the depth maps which can be used by the plugin.
   * \param poses The image's depth maps.
   */
  virtual void setDepthMaps(std::shared_ptr<CDataDepth> depthMaps) = 0;

  /*!
   * \brief Returns the computed 3D reconstruction.
   * \return The computed 3D reconstruction.
   *
   * This function should not be called before IAlgorithm::run has been called.
   */
  virtual std::shared_ptr<CDataFusion> getFusion() = 0;

  /*!
   * \}
   */
};

#endif // IFUSOR_H
