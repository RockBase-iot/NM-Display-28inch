#include "app/application.h"

void setup() {
    auto& app = NMDisplay28App::instance();
    app.init();
    app.begin();
}

void loop() {
    delay(1000);  // all logic runs inside FreeRTOS tasks
}
