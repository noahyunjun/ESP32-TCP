#include "driver_registry.h"
#include "drivers/meter_driver.h"

// Driver factory functions (defined in driver .cpp files)
const meter_driver_t* get_socomec_driver();
const meter_driver_t* get_janitza_driver();

#if defined(MODE_SOCOMEC_E14) && defined(MODE_JANITZA_UMG104)
#error "Select exactly one driver: MODE_SOCOMEC_E14 or MODE_JANITZA_UMG104"
#elif !defined(MODE_SOCOMEC_E14) && !defined(MODE_JANITZA_UMG104)
#error "No driver selected: define MODE_SOCOMEC_E14 or MODE_JANITZA_UMG104"
#endif

static const meter_driver_t* active_driver() {
#if defined(MODE_SOCOMEC_E14)
  return get_socomec_driver();
#elif defined(MODE_JANITZA_UMG104)
  return get_janitza_driver();
#endif
}

void driver_init() {
  const meter_driver_t* drv = active_driver();
  if (drv && drv->init) {
    drv->init();
  }
}

void driver_tick() {
  const meter_driver_t* drv = active_driver();
  if (drv && drv->tick) {
    drv->tick();
  }
}

const char* driver_name() {
  const meter_driver_t* drv = active_driver();
  return drv ? drv->name : "unknown";
}
