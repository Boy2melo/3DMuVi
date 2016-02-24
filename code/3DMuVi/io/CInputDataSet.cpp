#include "CInputDataSet.h"
#include "CImageIo.h"
#include <workflow/workflow/ccontextdatastore.h>

CInputDataSet::CInputDataSet(QUrl path) {
    auto strPath = path.toLocalFile();
    QDir inputDirectory(strPath);
    QStringList filters("*.png");

    inputDirectory.setNameFilters(filters);
    inputDirectory.setFilter(QDir::Filter::Files);
    inputDirectory.setSorting(QDir::SortFlag::Name);

    auto files = inputDirectory.entryList();

    CImageIo iio;
    uint32_t id = 0;

    foreach(const QString file, files) {
        QUrl imagePath(inputDirectory.filePath(file));
        auto imageCache = iio.load(imagePath);
        CImagePreviewItem qlwiCache(QIcon(QPixmap::fromImage(imageCache)), file, id);
        std::tuple<uint32_t, QImage, CImagePreviewItem> item(id, imageCache, qlwiCache);
        inputData.push_back(item);
        id++;
    }
}

std::vector<std::tuple<uint32_t, QImage, CImagePreviewItem>>* CInputDataSet::getInputImages() {
    return &inputData;
}

QString CInputDataSet::getDataType() const {
    return DT_INPUTIMAGES;
}

AStreamProvider* CInputDataSet::getStreamProvider() {
    // do not save input images
    return nullptr;
}
