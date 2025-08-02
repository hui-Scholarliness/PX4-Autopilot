/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "yufeng_demo.hpp"
#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/events.h>
#include <lib/matrix/matrix/math.hpp>

using namespace matrix;

yufeng_demo::yufeng_demo()
    : ModuleParams(nullptr),
      ScheduledWorkItem(MODULE_NAME,
                        px4::wq_configurations::nav_and_controllers) {
//   _sample_interval_s.update(0.01f);  // 100 Hz default
  parameters_update(true);
//   _tilt_limit_slew_rate.setSlewRate(.2f);
//   _takeoff_status_pub.advertise();
}

yufeng_demo::~yufeng_demo() { perf_free(_cycle_perf); }

bool yufeng_demo::init() {
  if (!_local_pos_sub.registerCallback()) {
    PX4_ERR("callback registration failed");
    return false;
  }

  _time_stamp_last_loop = hrt_absolute_time();
  ScheduleNow();

  return true;
}



void yufeng_demo::Run() {
  if (should_exit()) {
    _local_pos_sub.unregisterCallback();
    exit_and_cleanup();
    return;
  }

  // reschedule backup
  ScheduleDelayed(100_ms);

  parameters_update(false);

  perf_begin(_cycle_perf);
  vehicle_local_position_s vehicle_local_position;

  if (_local_pos_sub.update(&vehicle_local_position)) {
    if (_param_yu_feng_en.get()) {
      printf("hello sky!\r\n");
    } else {
      printf("hello land!\r\n");
    }
  }

  perf_end(_cycle_perf);
}

void yufeng_demo::parameters_update(bool force) {
  if (_parameter_update_sub.updated() || force) 
  {
    // clear update
    parameter_update_s pupdate;
    _parameter_update_sub.copy(&pupdate);

    // update parameters from storage
    ModuleParams::updateParams();

    float sample_freq_hz = 1.f / _sample_interval_s.mean();

    // velocity notch filter
    if ((_param_mpc_vel_nf_frq.get() > 0.f) &&
        (_param_mpc_vel_nf_bw.get() > 0.f)) {
      _vel_xy_notch_filter.setParameters(sample_freq_hz,
                                         _param_mpc_vel_nf_frq.get(),
                                         _param_mpc_vel_nf_bw.get());
      _vel_z_notch_filter.setParameters(sample_freq_hz,
                                        _param_mpc_vel_nf_frq.get(),
                                        _param_mpc_vel_nf_bw.get());

    } else {
      _vel_xy_notch_filter.disable();
      _vel_z_notch_filter.disable();
    }

    // velocity xy/z low pass filter
    if (_param_mpc_vel_lp.get() > 0.f) {
      _vel_xy_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_vel_lp.get());
      _vel_z_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_vel_lp.get());

    } else {
      // disable filtering
      _vel_xy_lp_filter.setAlpha(1.f);
      _vel_z_lp_filter.setAlpha(1.f);
    }

    // velocity derivative xy/z low pass filter
    if (_param_mpc_veld_lp.get() > 0.f) {
      _vel_deriv_xy_lp_filter.setCutoffFreq(sample_freq_hz,
                                            _param_mpc_veld_lp.get());
      _vel_deriv_z_lp_filter.setCutoffFreq(sample_freq_hz,
                                           _param_mpc_veld_lp.get());

    } else {
      // disable filtering
      _vel_deriv_xy_lp_filter.setAlpha(1.f);
      _vel_deriv_z_lp_filter.setAlpha(1.f);
    }
  }
}

int yufeng_demo::task_spawn(int argc, char *argv[])
{
  yufeng_demo *instance = new yufeng_demo();

  if (instance) {
    _object.store(instance);
    _task_id = task_id_is_work_queue;

    if (instance->init()) {
      return PX4_OK;
    }

  } else {
    PX4_ERR("alloc failed");
  }

  delete instance;
  _object.store(nullptr);
  _task_id = -1;

  return PX4_ERROR;
}

int yufeng_demo::custom_command(int argc, char *argv[])
{
  return print_usage("unknown command");
}

int yufeng_demo::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
yufeng_demo
)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("yufeng_demo", "controller");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

extern "C" __EXPORT int yufeng_demo_main(int argc, char *argv[])
{
  return yufeng_demo::main(argc, argv);
}
