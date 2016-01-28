#include "CInputDataSet.h"
#include "CImageIo.h"

std::vector<std::tuple<uint32_t, QImage, CImagePreviewItem>> inputData;

CInputDataSet::CInputDataSet(QUrl path)
{
    QDir inputDirectory(path.toString());
    QStringList filters ("*.png");

    inputDirectory.setNameFilters(filters);
    inputDirectory.setFilter(QDir::Filter::Files);
    inputDirectory.setSorting(QDir::SortFlag::Name);

    QStringList files = inputDirectory.entryList();

    CImageIo iio;
    uint32_t id = 0;

    foreach(const QString file, files)
    {
        QUrl imagePath(path.toString().append(file));
        QImage imageCache = iio.load(imagePath);
        CImagePreviewItem qlwiCache(QIcon(QPixmap::fromImage(imageCache)), file, id);
        std::tuple<uint32_t, QImage, CImagePreviewItem> item(id,imageCache,qlwiCache);
        inputData.push_back(item);
        id++;
    }
}

std::vector<std::tuple<uint32_t, QImage, CImagePreviewItem>>* CInputDataSet::getInputImages()
{
    return &inputData;
}

