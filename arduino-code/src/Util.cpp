#include <Util.h>

namespace Util {
    mbed::Timeout timeout;

    void onTimeout(bool &timeoutOccurred) {
        Serial.println("!!! A Timeout has occurred !!!");
        timeoutOccurred = true;
    }

    void beginTimeout(mbed::Timeout &timeout, bool &timeoutOccurred, float time) {
        timeoutOccurred = false;
        timeout.detach();
        timeout.attach(std::bind(&Util::onTimeout, std::ref(timeoutOccurred)), time);
    }
}
