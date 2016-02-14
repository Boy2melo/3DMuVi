#pragma once
#include "workflow/workflow/idatapacket.h"

class ExampleData :
    public IDataPacket {
public:
    ExampleData();
    ~ExampleData();

    QString getDataType() const override;
    void getMyData(void);
    void setMyData(void);
};

