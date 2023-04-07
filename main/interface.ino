void interface(char *buffer) {
  char cmd, subcmd, x;
  int i;
  double val;

  cmd = buffer[0];  
  switch (cmd) {
    case 'c':
      std::sscanf(buffer, "%c %c %d %lf", &cmd, &subcmd, &i, &val);
      if (LUMINAIRE == i) {
        switch (subcmd) {
          case 'k': /* Change gain K at luminaire i controller */            
            controller.set_k(val);
            Serial.println("ack");
            break;

          case 'i': /* Change time constant Ti at luminaire i controller */
            controller.set_ti(val);
            Serial.println("ack");
            break;

          case 'b': /* Change gain b at luminaire i controller */
            controller.set_b(val);
            Serial.println("ack");
            break;

          case 't': /* Change time constant Tt at luminaire i controller */
            controller.set_tt(val);
            Serial.println("ack");
            break;

          case 'r': /* Re-calibrate box gain */
            Serial.println("ack");
            box_gain = calibrate_gain();
            Serial.printf("Box gain: %lf\n", box_gain);
            break;
            
          default:
            Serial.println("err -> Invalid Command, please try again.");
            return; 
        }
      }
      break;    

    case 'd': /* Set directly the duty cycle of the LED at luminaire i */
      std::sscanf(buffer, "%c %d %lf", &cmd, &i, &val);
      if (LUMINAIRE == i) {
        serial_duty_cycle = val;
        analogWrite(LED_PIN, serial_duty_cycle * DAC_RANGE);
        Serial.println("ack");
      }
      else {
        float aux = (float) val;
        enqueue_message(i, msg_t::SET_DUTY_CYCLE, (uint8_t*) &aux, sizeof(aux));
      }
      break;

    case 'g':
      std::sscanf(buffer, "%c %c %d", &cmd, &subcmd, &i);
      switch (subcmd) {
        case 'd': /* Get current duty cycle at luminaire i */
          if (LUMINAIRE == i){
            if (controller.get_feedback())
              Serial.printf("d %d %lf\n", LUMINAIRE, controller.get_duty_cycle());
            else
              Serial.printf("d %d %lf\n", LUMINAIRE, serial_duty_cycle);
          }
          else
            enqueue_message(i, msg_t::GET_DUTY_CYCLE, nullptr, 0);
          break;

        case 'e': /* Get average energy consumption at desk <i> since the last system restart. */
          if (LUMINAIRE == i)
            Serial.printf("e %d %lf\n", LUMINAIRE, energy);
          else
            enqueue_message(i, msg_t::GET_ENERGY, nullptr, 0);
          break;
        
        case 'v': /* Get average visibility error at desk <i> since last system restart. */
          if (LUMINAIRE == i)
            Serial.printf("v %d %lf\n", LUMINAIRE, visibility_error / (double) iteration_counter);
          else
            enqueue_message(i, msg_t::GET_VISIBILITY_ERROR, nullptr, 0);
          break;

        case 'f': /* Get the average flicker error on desk <i> since the last system restart. */
          if (LUMINAIRE == i)
            Serial.printf("f %d %lf\n", LUMINAIRE, flicker_error / (double) iteration_counter);
          else
            enqueue_message(i, msg_t::GET_FLICKER_ERROR, nullptr, 0);
          break;

        case 'r': /* Get current illuminance reference at luminaire i */
          if (LUMINAIRE == i)
            Serial.printf("r %d %lf\n", LUMINAIRE, r);
          else
            enqueue_message(i, msg_t::GET_REFERENCE, nullptr, 0);
          break;
        
        case 'l': /* Get measured illuminance at luminaire i */
          if (LUMINAIRE == i)
            Serial.printf("l %d %lf\n", LUMINAIRE, lux_value);
          else
            enqueue_message(i, msg_t::GET_LUMINANCE, nullptr, 0);
          break;

        case 'o': /* Get current occupancy state at desk <i> */
          if (LUMINAIRE == i)
            Serial.printf("o %d %d\n", LUMINAIRE, controller.get_occupancy());
          else
            enqueue_message(i, msg_t::GET_OCCUPANCY, nullptr, 0);
          break;

        case 'a': /* Get anti-windup state at desk <i> */
          if (LUMINAIRE == i)
            Serial.printf("a %d %d\n", LUMINAIRE, controller.get_anti_windup());
          else
            enqueue_message(i, msg_t::GET_ANTI_WINDUP, nullptr, 0);
          break;

        case 'k': /* Get feedback state at desk <i> */
          if (LUMINAIRE == i)
            Serial.printf("k %d %d\n", LUMINAIRE, controller.get_feedback());
          else 
            enqueue_message(i, msg_t::GET_FEEDBACK, nullptr, 0);
          break;

        case 'x': /* Get current external illuminance at desk <i> */
          if (LUMINAIRE == i)
            Serial.printf("x %d %lf\n", LUMINAIRE, lux_value - box_gain * duty_cycle);
          else 
            enqueue_message(i, msg_t::GET_EXTERNAL_LUMINANCE, nullptr, 0);
          break;

        case 'p': /* Get instantaneous power consumption at desk <i> */
          if (LUMINAIRE == i)
            Serial.printf("p %d %lf\n", LUMINAIRE, (controller.get_u() / DAC_RANGE) * MAXIMUM_POWER);
          else 
            enqueue_message(i, msg_t::GET_POWER, nullptr, 0);
          break;  

        case 't': /* Get elapsed time since last restart */
          if (LUMINAIRE == i)
            Serial.printf("t %d %lf\n", LUMINAIRE, micros() * pow(10, -6));
          else 
            enqueue_message(i, msg_t::GET_ELAPSED_TIME, nullptr, 0);
          break;  
        
        case 'b': /* Get last minute buffer of variable <x> of desk <i>. <x> can be “l” or “d”. */
          std::sscanf(buffer, "%c %c %c %d", &cmd, &subcmd, &x, &i);
          if (LUMINAIRE == i) {
            switch (x) {
              case 'l':
                  buffer_l = !buffer_l;
                  buffer_read_size = last_minute_buffer.get_used_space();
                  buffer_read_counter = 0;
                  Serial.printf("b l %c", LUMINAIRE);
                break;

              case 'd':
                buffer_d = !buffer_d;
                buffer_read_size = last_minute_buffer.get_used_space();
                buffer_read_counter = 0;
                Serial.printf("b d %c", LUMINAIRE);
                break;

              default:
                Serial.println("err -> Invalid Command, please try again.");
               return;   
            } 
          }
          break;  

        case 'c': /* Get controller parameters at desk <i> */
          if (LUMINAIRE == i) {
            Serial.printf("Controller parameters at luminaire %d:\n", LUMINAIRE);
            Serial.printf("K = %lf\n", controller.get_k());
            Serial.printf("Ti = %lf\n", controller.get_ti());
            Serial.printf("b = %lf\n", controller.get_b());
            Serial.printf("Tt = %lf\n", controller.get_tt());
          }
          break;

        default:
          Serial.println("err -> Invalid Command, please try again.");
          return;           
      }
    break;

    case 'r': /* Set illuminance reference at luminaire i */
      if (std::sscanf(buffer, "%c %d %lf", &cmd, &i, &val) == 3) {
        if (LUMINAIRE == i) {
          r = val;
          Serial.println("ack");
        }
        else {
          float aux = (float) val;
          enqueue_message(i, msg_t::SET_REFERENCE, (uint8_t*) &aux, sizeof(aux));
        }
      }
      else {
        master_calibrate_routine();
      }      
      break;

    case 'o': /* Set current occupancy state at desk <i> */
      std::sscanf(buffer, "%c %d %lf", &cmd, &i, &val);
      if (LUMINAIRE == i) {
        controller.set_occupancy(val);
        Serial.println("ack");
      }
      else {
        float aux = (float) val;
        enqueue_message(i, msg_t::SET_OCCUPANCY, (uint8_t*) &aux, sizeof(aux)); 
      }
      break;

    case 'a': /* Set anti-windup state at desk <i> */
      std::sscanf(buffer, "%c %d %lf", &cmd, &i, &val);
      if (LUMINAIRE == i) {
        controller.set_anti_windup(val);
        Serial.println("ack");
      }
      else {
        float aux = (float) val;
        enqueue_message(i, msg_t::SET_ANTI_WINDUP, (uint8_t*) &aux, sizeof(aux)); 
      }
      break;

    case 'k': /* Set feedback on/off at desk <i> */
      std::sscanf(buffer, "%c %d %lf", &cmd, &i, &val);
      if (LUMINAIRE == i) {
        serial_duty_cycle = controller.get_u() / DAC_RANGE;
        controller.set_feedback(val);
        Serial.println("ack");
      }
      else {
        float aux = (float) val;
        enqueue_message(i, msg_t::SET_FEEDBACK, (uint8_t*) &aux, sizeof(aux));
      }
      break;

    case 's': /* Start stream of real-time variable <x> of desk <i>. <x> can be “l”, “d” or "j". */
      std::sscanf(buffer, "%c %c %d", &cmd, &x, &i);
      if (LUMINAIRE == i) {
        if (x == 'l')
          stream_l = 1;
        else if (x == 'd')
          stream_d = 1;
        else if (x == 'j')
          stream_j = 1;
      }
      break;    
    
    case 'S': /* Stop stream of real-time variable <x> of desk <i>. <x> can be “l”, “d” or "j". */
      std::sscanf(buffer, "%c %c %d", &cmd, &x, &i);
      if (LUMINAIRE == i) {
        if (x == 'l')
          stream_l = 0;
        else if (x == 'd')
          stream_d = 0;   
        else if (x == 'j')
          stream_j = 0;  
      }
      Serial.println("ack");
      break;  

    case 'v': /* Visualization with serial plotter */
      std::sscanf(buffer, "%c %d", &cmd, &i);
      if (LUMINAIRE == i) {
        visualization = !visualization;
        Serial.println("ack");
      }
      else
        Serial.println("err");
      break;
    
    case 'm': /* Set mode of operation */
      std::sscanf(buffer, "%c %c %d", &cmd, &subcmd, &i);
      if (LUMINAIRE == i)
        controller.set_modeOp(subcmd, i);
      Serial.println("ack");
      break;      

    default:
      Serial.println("err -> Invalid Command, please try again.");
      return;      
  }
}