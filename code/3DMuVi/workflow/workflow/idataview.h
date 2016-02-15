#ifndef IDATAVIEW_H
#define IDATAVIEW_H

class IDataView {
public:
    IDataView();

    template<typename T>
    void applyData(T *data) {}
};

#endif // IDATAVIEW_H
