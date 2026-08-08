// Auto-generated ESP-IDF entry shim. Put application logic in firmware/app/app.c.
#include "app.h"
#include "firmgen_telemetry.h"
#include "firmgen_proto_monitor.h"

void app_main(void)
{
    firmgen_telemetry_start();
    firmgen_proto_monitor_start();
    app_start();
}
