
#ifndef _CLOGINIT_H
#define _CLOGINIT_H
class CLogInit {
public: 
    
    ostringstream getInfostream();
    
    ostringstream getErrorstream();
    
    ostringstream getDebugstream();
    
    ostringstream getWarningstream();
private: 
    ostringstream infostream;
    ostringstream errorstream;
    ostringstream debugstream;
    ostringstream warningstream;
};

#endif //_CLOGINIT_H
