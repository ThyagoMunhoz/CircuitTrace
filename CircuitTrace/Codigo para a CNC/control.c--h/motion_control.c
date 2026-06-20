

void mc_line(float *target, plan_line_data_t *pl_data)
{
  if (bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE)) {
    if (sys.state != STATE_JOG) { limits_soft_check(target); }
  }

  if (sys.state == STATE_CHECK_MODE) { return; }

  do {
    protocol_execute_realtime(); 
    if (sys.abort) { return; } 
    if ( plan_check_full_buffer() ) { protocol_auto_cycle_start(); } 
    else { break; }
  } while (1);

  if (plan_buffer_line(target, pl_data) == PLAN_EMPTY_BLOCK) {
    if (bit_istrue(settings.flags,BITFLAG_LASER_MODE)) {
      if (pl_data->condition & PL_COND_FLAG_SPINDLE_CW) {
        spindle_sync(PL_COND_FLAG_SPINDLE_CW, pl_data->spindle_speed);
      }
    }
  }
}

void mc_arc(float *target, plan_line_data_t *pl_data, float *position, float *offset, float radius,
  uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc)
{
  float center_axis0 = position[axis_0] + offset[axis_0];
  float center_axis1 = position[axis_1] + offset[axis_1];
  float r_axis0 = -offset[axis_0];  
  float r_axis1 = -offset[axis_1];
  float rt_axis0 = target[axis_0] - center_axis0;
  float rt_axis1 = target[axis_1] - center_axis1;

  float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
  if (is_clockwise_arc) { 
    if (angular_travel >= -ARC_ANGULAR_TRAVEL_EPSILON) { angular_travel -= 2*M_PI; }
  } else {
    if (angular_travel <= ARC_ANGULAR_TRAVEL_EPSILON) { angular_travel += 2*M_PI; }
  }

  uint16_t segments = floor(fabs(0.5*angular_travel*radius)/
                          sqrt(settings.arc_tolerance*(2*radius - settings.arc_tolerance)) );

  if (segments) {
    if (pl_data->condition & PL_COND_FLAG_INVERSE_TIME) { 
      pl_data->feed_rate *= segments; 
      bit_false(pl_data->condition,PL_COND_FLAG_INVERSE_TIME); 
    }
    
    float theta_per_segment = angular_travel/segments;
    float linear_per_segment = (target[axis_linear] - position[axis_linear])/segments;

    float cos_T = 2.0 - theta_per_segment*theta_per_segment;
    float sin_T = theta_per_segment*0.16666667*(cos_T + 4.0);
    cos_T *= 0.5;

    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    uint8_t count = 0;

    for (i = 1; i<segments; i++) { 

      if (count < N_ARC_CORRECTION) {
        r_axisi = r_axis0*sin_T + r_axis1*cos_T;
        r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
        r_axis1 = r_axisi;
        count++;
      } else {
        cos_Ti = cos(i*theta_per_segment);
        sin_Ti = sin(i*theta_per_segment);
        r_axis0 = -offset[axis_0]*cos_Ti + offset[axis_1]*sin_Ti;
        r_axis1 = -offset[axis_0]*sin_Ti - offset[axis_1]*cos_Ti;
        count = 0;
      }

      position[axis_0] = center_axis0 + r_axis0;
      position[axis_1] = center_axis1 + r_axis1;
      position[axis_linear] += linear_per_segment;

      mc_line(position, pl_data);

      if (sys.abort) { return; }
    }
  }
  mc_line(target, pl_data);
}

void mc_dwell(float seconds)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize();
  delay_sec(seconds, DELAY_MODE_DWELL);
}

void mc_homing_cycle(uint8_t cycle_mask)
{
  #ifdef LIMITS_TWO_SWITCHES_ON_AXES
    if (limits_get_state()) {
      mc_reset(); 
      system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT);
      return;
    }
  #endif

  limits_disable(); 

  #ifdef HOMING_SINGLE_AXIS_COMMANDS
    if (cycle_mask) { limits_go_home(cycle_mask); } 
    else
  #endif
  {
    limits_go_home(HOMING_CYCLE_0);  
    #ifdef HOMING_CYCLE_1
      limits_go_home(HOMING_CYCLE_1);  
    #endif
    #ifdef HOMING_CYCLE_2
      limits_go_home(HOMING_CYCLE_2);  
    #endif
  }

  protocol_execute_realtime(); 
  if (sys.abort) { return; } 

  gc_sync_position();
  plan_sync_position();

  limits_init();
}

uint8_t mc_probe_cycle(float *target, plan_line_data_t *pl_data, uint8_t parser_flags)
{
  if (sys.state == STATE_CHECK_MODE) { return(GC_PROBE_CHECK_MODE); }

  protocol_buffer_synchronize();
  if (sys.abort) { return(GC_PROBE_ABORT); } 

  uint8_t is_probe_away = bit_istrue(parser_flags,GC_PARSER_PROBE_IS_AWAY);
  uint8_t is_no_error = bit_istrue(parser_flags,GC_PARSER_PROBE_IS_NO_ERROR);
  sys.probe_succeeded = false; 
  probe_configure_invert_mask(is_probe_away);

  if ( probe_get_state() ) { 
    system_set_exec_alarm(EXEC_ALARM_PROBE_FAIL_INITIAL);
    protocol_execute_realtime();
    probe_configure_invert_mask(false); 
    return(GC_PROBE_FAIL_INIT); 
  }

  mc_line(target, pl_data);

  sys_probe_state = PROBE_ACTIVE;

  system_set_exec_state_flag(EXEC_CYCLE_START);
  do {
    protocol_execute_realtime();
    if (sys.abort) { return(GC_PROBE_ABORT); } 
  } while (sys.state != STATE_IDLE);

  if (sys_probe_state == PROBE_ACTIVE) {
    if (is_no_error) { memcpy(sys_probe_position, sys_position, sizeof(sys_position)); }
    else { system_set_exec_alarm(EXEC_ALARM_PROBE_FAIL_CONTACT); }
  } else {
    sys.probe_succeeded = true; 
  }
  sys_probe_state = PROBE_OFF; 
  probe_configure_invert_mask(false); 
  protocol_execute_realtime();   

  st_reset(); 
  plan_reset(); 
  plan_sync_position(); 

  #ifdef MESSAGE_PROBE_COORDINATES
    report_probe_parameters();
  #endif

  if (sys.probe_succeeded) { return(GC_PROBE_FOUND); } 
  else { return(GC_PROBE_FAIL_END); } 
}

#ifdef PARKING_ENABLE
  void mc_parking_motion(float *parking_target, plan_line_data_t *pl_data)
  {
    if (sys.abort) { return; } 

    uint8_t plan_status = plan_buffer_line(parking_target, pl_data);

    if (plan_status) {
      bit_true(sys.step_control, STEP_CONTROL_EXECUTE_SYS_MOTION);
      bit_false(sys.step_control, STEP_CONTROL_END_MOTION); 
      st_parking_setup_buffer(); 
      st_prep_buffer();
      st_wake_up();
      do {
        protocol_exec_rt_system();
        if (sys.abort) { return; }
      } while (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION);
      st_parking_restore_buffer(); 
    } else {
      bit_false(sys.step_control, STEP_CONTROL_EXECUTE_SYS_MOTION);
      protocol_exec_rt_system();
    }
  }
#endif

#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
  void mc_override_ctrl_update(uint8_t override_state)
  {
    protocol_buffer_synchronize();
    if (sys.abort) { return; }
    sys.override_ctrl = override_state;
  }
#endif

void mc_reset()
{
  if (bit_isfalse(sys_rt_exec_state, EXEC_RESET)) {
    system_set_exec_state_flag(EXEC_RESET);

    spindle_stop();
    coolant_stop();

    if ((sys.state & (STATE_CYCLE | STATE_HOMING | STATE_JOG)) ||
    		(sys.step_control & (STEP_CONTROL_EXECUTE_HOLD | STEP_CONTROL_EXECUTE_SYS_MOTION))) {
      if (sys.state == STATE_HOMING) { 
        if (!sys_rt_exec_alarm) {system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_RESET); }
      } else { system_set_exec_alarm(EXEC_ALARM_ABORT_CYCLE); }
      st_go_idle(); 
    }
  }
}