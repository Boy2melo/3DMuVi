#ifndef IGUIDATAVIEW_H
#define IGUIDATAVIEW_H

/*!
\brief Interface for data views in the GUI.
\author Stefan Wolf

This interface schould be implemented by all views in the GUI to get a consistent way to access
them.
*/
class IGuiDataView
{
public:
  /*!
  \brief Called when data view is activated.

  This method will be called when the data view gets visible. Implementing classes should then
  do all necessary tasks to get the program in a correct state. Especially, a signal should be sent
  indicating which images are relevant now.
  */
  virtual void activate() = 0;
};

#endif // IGUIDATAVIEW_H
