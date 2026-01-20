#ifndef METER_DRIVER_H
#define METER_DRIVER_H

typedef void (*driver_init_fn)();
typedef void (*driver_tick_fn)();

struct meter_driver_t {
  const char* name;
  driver_init_fn init;
  driver_tick_fn tick;
};

#endif
