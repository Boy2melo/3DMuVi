/**
 * Project Untitled
 */


#include "CLogInit.h"
/**
 * CLogInit implementation
 */


/**
 * @return *ostringstream
 */
ostringstream CLogInit::getInfostream () {
    return infostream;
}

/**
 * @return *ostringstream
 */
ostringstream CLogInit::getErrorstream() {
    return null;
}

/**
 * @return *ostringstream
 */
ostringstream CLogInit::getDebugstream() {
    return null;
}

/**
 * @return *ostringstream
 */
ostringstream CLogInit::getWarningstream() {
    return null;
}
