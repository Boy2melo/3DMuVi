#ifndef IDATAVIEW_H
#define IDATAVIEW_H

class IDataView
{
public:
    IDataView();

    template<typename T>
    void ApplyData(T& data);
};

#endif // IDATAVIEW_H
