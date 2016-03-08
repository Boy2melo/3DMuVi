#include <settings/CGlobalSettingController.h>
class CTestCGlobalSettingController : public QObject
{
Q_OBJECT
private slots:
    void initTestCase();
    /*!
     * \brief testgetSetting test the getSetting method
     */
    void testgetSetting();
    /*!
     * \brief testsetSetting test the setSetting method
     */
    void testsetSetting();
    /*!
     * \brief testexport test the export method
     */
    void testexport();
private:
  CGlobalSettingController controller;
};
