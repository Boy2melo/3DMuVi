#include "CTestCImageTab.h"
#include <QUrl>
#include <QTemporaryDir>
#include <QTemporaryFile>
#include "workflow/workflow/fourphase/cfourphaseworkflow.h"
#include "gui/CMainWindow.h"
#include "gui/CImagePreviewWidget.h"

void CTestCImageTab::test2Images() {
    QTemporaryDir tempDir;
    if (tempDir.isValid()) {
        QDir path = QDir(tempDir.path());
        createImages(path, 2, "png");

        validateView(path, 2);
    }

    tempDir.remove();
}


void CTestCImageTab::test0Images() {
    QTemporaryDir tempDir;
    if (tempDir.isValid()) {
        QDir path = QDir(tempDir.path());
        createImages(path, 0, "png");

        validateView(path, 0);
    }

    tempDir.remove();
}

void CTestCImageTab::test500Images() {
    QTemporaryDir tempDir;
    if (tempDir.isValid()) {
        QDir path = QDir(tempDir.path());
        createImages(path, 500, "png");

        validateView(path, 500);
    }

    tempDir.remove();
}

void CTestCImageTab::testTypes() {
    QTemporaryDir tempDir;
    if (tempDir.isValid()) {
        QDir path = QDir(tempDir.path());
        createImages(path, 2, "png");
        createImages(path, 2, "bmp");
        createImages(path, 2, "jpeg");

        validateView(path, 6);
    }

    tempDir.remove();
}

void CTestCImageTab::validateView(QDir dir, int expected) {
    CMainWindow mw;
    CImagePreviewWidget *preview;
    auto workflow = new CFourPhaseWorkflow();

    for (QWidget *w : QApplication::allWidgets()) {
        preview = qobject_cast<CImagePreviewWidget*>(w);

        if (preview) {
            if (preview->objectName() != "imagePreviewWidget") {
                preview = nullptr;
            } else {
                break;
            }
        }
    }

    if(!preview) {
        QFAIL("Could not find image preview widget by name. Please fix ui or test");
    }

    mw.setLoadImage(QUrl(dir.absolutePath()));
    
    QCOMPARE(preview->count(), 0); // No workflow loaded

    mw.setWorkflow(workflow);
    mw.setLoadImage(QUrl(dir.absolutePath()));

    QCOMPARE(preview->count(), expected);
}

void CTestCImageTab::createImages(QDir path, uint images, const char ext[]) {
    uchar* data = new uchar[4]{ 0xFF, 0xFF, 0xFF, 0xFF };

    for (uint i = 0; i < images; i++) {
        QImage img = QImage(data, 1, 1, QImage::Format_ARGB32);
        QString file = path.filePath(QString::number(i) + "." + ext);
        img.save(file, ext);
    }

    delete[] data;
}
