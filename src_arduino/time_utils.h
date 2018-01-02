#ifndef TIME_UTILS_H
#define TIME_UTILS_H

static uint32_t last_micros_time = 0;
static double collective_micros_wraparound_offset = 0.0;
static double local_to_global_time_offset = 0.0;
static double local_to_global_time_scaling = 1.0;

double get_current_time(){
  uint32_t new_micros = micros();
  if (last_micros_time > 1000*1000*1000 && (new_micros < last_micros_time - 1000*1000*1000)){
    collective_micros_wraparound_offset += 4294.967296; // 2^32 / 1000 / 1000. 
  }
  last_micros_time = new_micros;
  return local_to_global_time_scaling * (((double)new_micros) / 1000. / 1000. + collective_micros_wraparound_offset) + local_to_global_time_offset;
}

#endif
