#include <sstream>
#include <set>
#include <map>
#include <numeric>
#include <algorithm>
#include <hardware/flash.h>
#include "mcp2515.h"
#include "quickselect.h"
#include "controller.h"
#include "circular_buffer.h"

#define DIFFERENCE(a, b) ((a) > (b) ? (a) - (b) : (b) - (a))
#define BROADCAST 0

// Constants
const int DAC_RANGE = 4096;
const int DAC_BITS = 12;
const int LED_PIN = 15;
const int ADC_PIN = A0;
const unsigned long PING_TIMER = 5000;
const double Vcc = 3.3;
const double MAXIMUM_POWER = 2.65 * 5.8e-3;
const int sampInterval = 10;
const int CALIBRATION_START_DELAY = 5000;

const int R = 10e3;
const double b = 6.1521825181113625;
const double m = -0.8;

const int MEDIAN_FILTER_SIZE = 5;
int last_measurements[MEDIAN_FILTER_SIZE] = {1, 1, 1, 1, 1};

// Luminaire ID
uint8_t LUMINAIRE;

enum inter_core_cmds
{
  // From core1 to core0: contains data read (16 bit)
  ICC_READ_DATA = 1,
  // From core0 to core1: contains data to write (16 bit)
  ICC_WRITE_DATA = 2,
  // From core1 to core0: contains regs CANINTF, EFLG
  ICC_ERROR_DATA = 3
};

// Globals
int counter = 0;
double box_gain;
std::stringstream command_ss;
char command_buffer[64] = "";
Controller controller;
int incomingByte = 0;
double r = 0;
double lux_value;
double duty_cycle = 0.0;
int iteration_counter = 0;
double energy = 0.0;
double visibility_error = 0.0;
double flicker_error = 0.0;
double prev_duty_cycle_1 = 0.0;
double prev_duty_cycle_2 = 0.0;
int stream_l = 0, stream_d = 0, stream_j = 0;
CircularBuffer<6000> last_minute_buffer;
buffer_data data;
int buffer_d = 0;
int buffer_l = 0;
int buffer_read_size = 0;
int buffer_read_counter = 0;
double serial_duty_cycle = 0;
int visualization = 0;
MCP2515 can0{spi0, 17, 19, 16, 18, 10000000};
unsigned long time_since_last_ping = 0;

std::set<uint8_t> other_luminaires;
std::vector<uint8_t> luminaire_ids;
std::set<uint8_t> ready_luminaires;
std::map<int, float> coupling_gains;
bool is_calibrating_as_master = false, is_calibrating = false;
uint8_t calibration_master = 0, calibrating_luminaire = 0;
std::size_t total_calibrations = 0;

enum calibration_stage_t : uint8_t
{
  WAIT_FOR_ACK = 0,
  CHANGE_STATE,
  CALIBRATING,
  DONE
};

calibration_stage_t calibration_stage, next_stage;

enum msg_t : uint8_t
{
  PING = 0,
  OFF,
  ON,
  END,
  ACK,
  READY_TO_READ,
  SET_DUTY_CYCLE,
  GET_DUTY_CYCLE,
  DUTY_CYCLE_VALUE,
  SET_REFERENCE,
  GET_REFERENCE, 
  REFERENCE_VALUE,
  GET_LUMINANCE,
  LUMINANCE_VALUE,
  SET_OCCUPANCY,
  GET_OCCUPANCY, 
  OCCUPANCY_VALUE,
  SET_ANTI_WINDUP,
  GET_ANTI_WINDUP, 
  ANTI_WINDUP_VALUE,
  SET_FEEDBACK,
  GET_FEEDBACK,
  FEEDBACK_VALUE,
  GET_EXTERNAL_LUMINANCE,
  EXTERNAL_LUMINANCE_VALUE,
  GET_POWER,
  POWER_VALUE,
  GET_ELAPSED_TIME,
  ELAPSED_TIME_VALUE,
  GET_ENERGY,
  ENERGY_VALUE,
  GET_VISIBILITY_ERROR,
  VISIBILITY_ERROR_VALUE,
  GET_FLICKER_ERROR,
  FLICKER_ERROR_VALUE,
  SET_LOWER_BOUND_OCCUPIED,
  GET_LOWER_BOUND_OCCUPIED,
  LOWER_BOUND_OCCUPIED_VALUE,
  SET_LOWER_BOUND_UNOCCUPIED,
  GET_LOWER_BOUND_UNOCCUPIED,
  LOWER_BOUND_UNOCCUPIED_VALUE,
  GET_LOWER_BOUND,
  LOWER_BOUND_VALUE,
  SET_COST,
  GET_COST,
  COST_VALUE
};
char message_type_translations[][29] = {"PING", "OFF", "ON", "END", "ACK", "READY_TO_READ", "SET_DUTY_CYCLE", "GET_DUTY_CYCLE", "DUTY_CYCLE_VALUE", "SET_REFERENCE", "GET_REFERENCE", "REFERENCE_VALUE", "GET_LUMINANCE", "LUMINANCE_VALUE", "SET_OCCUPANCY", "GET_OCCUPANCY", "OCCUPANCY_VALUE", "SET_ANTI_WINDUP", "GET_ANTI_WINDUP", "ANTI_WINDUP_VALUE", "SET_FEEDBACK", "GET_FEEDBACK", "FEEDBACK_VALUE", "GET_EXTERNAL_LUMINANCE", "EXTERNAL_LUMINANCE_VALUE", "GET_POWER", "POWER_VALUE", "GET_ELAPSED_TIME", "ELAPSED_TIME_VALUE", "GET_ENERGY", "ENERGY_VALUE", "GET_VISIBILITY_ERROR", "VISIBILITY_ERROR_VALUE", "GET_FLICKER_ERROR", "FLICKER_ERROR_VALUE", "SET_LOWER_BOUND_OCCUPIED", "GET_LOWER_BOUND_OCCUPIED", "LOWER_BOUND_OCCUPIED_VALUE", "SET_LOWER_BOUND_UNOCCUPIED", "GET_LOWER_BOUND_UNOCCUPIED", "LOWER_BOUND_UNOCCUPIED_VALUE", "GET_LOWER_BOUND", "LOWER_BOUND_VALUE", "SET_COST", "GET_COST", "COST_VALUE"};

double adc2resistance(int adc_value)
{
  /* V_ADC = adc_value / DAC_RANGE * VCC; R_LDR = VCC / V_ADC * R - R */
  return R * (DAC_RANGE / ((double)adc_value) - 1);
}

double resistance2lux(double R_LDR)
{
  return pow(10, (log10(R_LDR) - b) / m);
}

double adc2lux(int adc_value)
{
  return resistance2lux(adc2resistance(adc_value));
}

void write_into_filter(int value)
{
  memmove(last_measurements, last_measurements + 1, sizeof(int) * 4);
  last_measurements[4] = value;
}

int read_from_filter()
{
  static int buffer[MEDIAN_FILTER_SIZE];
  memcpy(buffer, last_measurements, sizeof(last_measurements));
  return quick_select(buffer, MEDIAN_FILTER_SIZE);
}

/* Get box gain to then compute the external luminance */
double calibrate_gain()
{
  double u1 = 0, u2 = 1, y1, y2;

  analogWrite(LED_PIN, 0);
  delay(5000);
  Serial.println("Starting calibration, please wait ~12 seconds...");

  // LED turned off
  y1 = adc2lux(analogRead(ADC_PIN));
  delay(1000);

  // LED turned on
  analogWrite(LED_PIN, DAC_RANGE);
  delay(10000);
  y2 = adc2lux(analogRead(ADC_PIN));
  delay(1000);

  return (y2 - y1) / (u2 - u1);
}

void setup()
{
  uint8_t pico_flash_id[8];
  rp2040.idleOtherCore();
  flash_get_unique_id(pico_flash_id);
  rp2040.resumeOtherCore();
  LUMINAIRE = pico_flash_id[7];

  Serial.begin(115200);
  analogReadResolution(DAC_BITS);
  analogWriteRange(DAC_RANGE);
  analogWriteFreq(60000);
  controller.set_controller(LUMINAIRE);

  // Box gain calibration
  // box_gain = calibrate_gain();
  Serial.printf("Box gain: %lf\n", box_gain);
  time_since_last_ping = millis();
}

void setup1()
{
  uint8_t pico_flash_id[8];
  rp2040.idleOtherCore();
  flash_get_unique_id(pico_flash_id);
  rp2040.resumeOtherCore();
  LUMINAIRE = pico_flash_id[7];

  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setFilterMask(MCP2515::MASK0, 0, 0x000000ff);
  can0.setFilter(MCP2515::RXF0, 0, 0);
  can0.setFilter(MCP2515::RXF1, 0, LUMINAIRE);
  can0.setFilterMask(MCP2515::MASK1, 0, 0x000000ff);
  can0.setFilter(MCP2515::RXF2, 0, 0);
  can0.setFilter(MCP2515::RXF3, 0, LUMINAIRE);
  can0.setNormalMode();
}

void serial_command()
{
  const int BUFFER_SIZE = 128;
  static char buffer[BUFFER_SIZE];
  static int buffer_pos = 0;

  can_frame *frm;
  while (rp2040.fifo.available())
  {
    if (!rp2040.fifo.pop_nb((uint32_t *)&frm))
      break;

    msg_t message_type = (msg_t)frm->data[1];
    uint8_t sender = frm->data[0];
    uint8_t *data = &frm->data[2];

    if (message_type < sizeof(message_type_translations) && message_type != msg_t::PING)
      Serial.printf("Received message of type %s from %d\n", message_type_translations[message_type], sender);
    else if (message_type != msg_t::PING)
      Serial.printf("Received message of unknown type (%d) from %d\n", message_type, sender);

    float value = 0;
    int id_active = 0;
    double lux_value = 0;

    switch (message_type)
    {
    case msg_t::PING:
      if (other_luminaires.insert(sender).second)
        Serial.printf("Added %d as a new neighbour\n", sender);
      break;

    case msg_t::OFF:
      controller.set_feedback(false);
      analogWrite(LED_PIN, 0);
      enqueue_message(sender, msg_t::ACK, nullptr, 0);
      break;

    case msg_t::ON:
      controller.set_feedback(false);
      analogWrite(LED_PIN, 4095);
      enqueue_message(sender, msg_t::ACK, nullptr, 0);
      break;

    case msg_t::END:
      if (!is_calibrating_as_master)
      {
        controller.set_feedback(true);
        print_map(coupling_gains);
        enqueue_message(sender, msg_t::ACK, nullptr, 0);
      }
      else
        ready_luminaires.insert(sender);
      break;

    case msg_t::ACK:
      if (is_calibrating || is_calibrating_as_master)
        ready_luminaires.insert(sender);
      else
        Serial.printf("ack from %d\n", sender);
      break;

    case msg_t::READY_TO_READ:
      memcpy(&id_active, data, sizeof(id_active));
      lux_value = adc2lux(read_from_filter());
      if (id_active == -1 || coupling_gains.count(-1) == 0)
        coupling_gains[id_active] = lux_value;
      else
        coupling_gains[id_active] = lux_value - coupling_gains[-1];      
      break;
    
    case msg_t::SET_DUTY_CYCLE:
      memcpy(&value, data, sizeof(value));
      serial_duty_cycle = (double)value;
      analogWrite(LED_PIN, value * DAC_RANGE);
      enqueue_message(sender, msg_t::ACK, nullptr, 0);
      break;

    case msg_t::GET_DUTY_CYCLE:
      if (controller.get_feedback())
        value = (float)controller.get_duty_cycle();
      else
        value = (float)serial_duty_cycle;
      enqueue_message(sender, msg_t::DUTY_CYCLE_VALUE, (uint8_t *)&value, sizeof(value));
      break;

    case msg_t::DUTY_CYCLE_VALUE:
      memcpy(&value, data, sizeof(value));
      Serial.printf("d %d %f\n", sender, value);
      break;

    case msg_t::SET_REFERENCE:
      memcpy(&value, data, sizeof(value));
      r = (double)value;
      enqueue_message(sender, msg_t::ACK, nullptr, 0);
      break;

    case msg_t::GET_REFERENCE:
      value = (float)r;
      enqueue_message(sender, msg_t::REFERENCE_VALUE, (uint8_t *)&value, sizeof(value));
      break;

    case msg_t::REFERENCE_VALUE:
      memcpy(&value, data, sizeof(value));
      Serial.printf("r %d %f\n", sender, value);
      break;

    case msg_t::GET_LUMINANCE:
      value = (float)adc2lux(read_from_filter());
      enqueue_message(sender, msg_t::LUMINANCE_VALUE, (uint8_t *)&value, sizeof(value));
      break;

    case msg_t::LUMINANCE_VALUE:
      memcpy(&value, data, sizeof(value));
      Serial.printf("l %d %f\n", sender, value);
      break;

    case msg_t::SET_OCCUPANCY:
      memcpy(&value, data, sizeof(value));
      controller.set_occupancy((int)value);
      enqueue_message(sender, msg_t::ACK, nullptr, 0);
      break;

    case msg_t::GET_OCCUPANCY:
      value = (float)controller.get_occupancy();
      enqueue_message(sender, msg_t::OCCUPANCY_VALUE, (uint8_t *)&value, sizeof(value));
      break;

    case msg_t::OCCUPANCY_VALUE:
      memcpy(&value, data, sizeof(value));
      Serial.printf("o %d %d\n", sender, (int)value);
      break;

    case msg_t::SET_ANTI_WINDUP:
      memcpy(&value, data, sizeof(value));
      controller.set_anti_windup((int)value);
      enqueue_message(sender, msg_t::ACK, nullptr, 0);
      break;

    case msg_t::GET_ANTI_WINDUP:
      value = (float)controller.get_anti_windup();
      enqueue_message(sender, msg_t::ANTI_WINDUP_VALUE, (uint8_t *)&value, sizeof(value));
      break;

    case msg_t::ANTI_WINDUP_VALUE:
      memcpy(&value, data, sizeof(value));
      Serial.printf("a %d %d\n", sender, (int)value);
      break;

    case msg_t::SET_FEEDBACK:
      memcpy(&value, data, sizeof(value));
      serial_duty_cycle = controller.get_u() / DAC_RANGE;
      controller.set_feedback((int)value);
      enqueue_message(sender, msg_t::ACK, nullptr, 0);
      break;

    case msg_t::GET_FEEDBACK:
      value = (float)controller.get_feedback();
      enqueue_message(sender, msg_t::FEEDBACK_VALUE, (uint8_t *)&value, sizeof(value));
      break;
      
    case msg_t::FEEDBACK_VALUE:
      memcpy(&value, data, sizeof(value));
      Serial.printf("k %d %d\n", sender, (int)value);
      break;

    case msg_t::GET_EXTERNAL_LUMINANCE:
      if (controller.get_feedback())
        value = (float)(adc2lux(read_from_filter()) - coupling_gains[LUMINAIRE] * controller.get_duty_cycle());
      else
        value = (float)(adc2lux(read_from_filter()) - coupling_gains[LUMINAIRE] * serial_duty_cycle);
      enqueue_message(sender, msg_t::EXTERNAL_LUMINANCE_VALUE, (uint8_t *)&value, sizeof(value));
      break;
      
    case msg_t::EXTERNAL_LUMINANCE_VALUE:
      memcpy(&value, data, sizeof(value));
      Serial.printf("x %d %f\n", sender, value);
      break;

    case msg_t::GET_POWER:
      value = (float)((controller.get_u() / DAC_RANGE) * MAXIMUM_POWER);
      enqueue_message(sender, msg_t::POWER_VALUE, (uint8_t *)&value, sizeof(value));
      break;
      
    case msg_t::POWER_VALUE:
      memcpy(&value, data, sizeof(value));
      Serial.printf("p %d %f\n", sender, value);
      break;

    case msg_t::GET_ELAPSED_TIME:
      value = (float)(micros() * pow(10, -6));
      enqueue_message(sender, msg_t::ELAPSED_TIME_VALUE, (uint8_t *)&value, sizeof(value));
      break;
      
    case msg_t::ELAPSED_TIME_VALUE:
      memcpy(&value, data, sizeof(value));
      Serial.printf("t %d %f\n", sender, value);
      break;

    case msg_t::GET_ENERGY:
      value = (float)energy;
      enqueue_message(sender, msg_t::ENERGY_VALUE, (uint8_t *)&value, sizeof(value));
      break;
      
    case msg_t::ENERGY_VALUE:
      memcpy(&value, data, sizeof(value));
      Serial.printf("e %d %f\n", sender, value);
      break;

    case msg_t::GET_VISIBILITY_ERROR:
      value = (float)(visibility_error / (double) iteration_counter);
      enqueue_message(sender, msg_t::VISIBILITY_ERROR_VALUE, (uint8_t *)&value, sizeof(value));
      break;
      
    case msg_t::VISIBILITY_ERROR_VALUE:
      memcpy(&value, data, sizeof(value));
      Serial.printf("v %d %f\n", sender, value);
      break;

    case msg_t::GET_FLICKER_ERROR:
      value = (float)(flicker_error / (double) iteration_counter);
      enqueue_message(sender, msg_t::FLICKER_ERROR_VALUE, (uint8_t *)&value, sizeof(value));
      break;
      
    case msg_t::FLICKER_ERROR_VALUE:
      memcpy(&value, data, sizeof(value));
      Serial.printf("f %d %f\n", sender, value);
      break;

    case msg_t::SET_LOWER_BOUND_OCCUPIED:
      memcpy(&value, data, sizeof(value));
      controller.set_lower_bound_Occupied((double)value);
      enqueue_message(sender, msg_t::ACK, nullptr, 0);
      break;

    case msg_t::GET_LOWER_BOUND_OCCUPIED:
      value = (float)controller.get_lower_bound_Occupied();
      enqueue_message(sender, msg_t::LOWER_BOUND_OCCUPIED_VALUE, (uint8_t *)&value, sizeof(value));
      break;
      
    case msg_t::LOWER_BOUND_OCCUPIED_VALUE:
      memcpy(&value, data, sizeof(value));
      Serial.printf("O %d %f\n", sender, value);
      break;

    case msg_t::SET_LOWER_BOUND_UNOCCUPIED:
      memcpy(&value, data, sizeof(value));
      controller.set_lower_bound_Unoccupied((double)value);
      enqueue_message(sender, msg_t::ACK, nullptr, 0);
      break;

    case msg_t::GET_LOWER_BOUND_UNOCCUPIED:
      value = (float)controller.get_lower_bound_Unoccupied();
      enqueue_message(sender, msg_t::LOWER_BOUND_UNOCCUPIED_VALUE, (uint8_t *)&value, sizeof(value));
      break;
      
    case msg_t::LOWER_BOUND_UNOCCUPIED_VALUE:
      memcpy(&value, data, sizeof(value));
      Serial.printf("U %d %f\n", sender, value);
      break;

    case msg_t::GET_LOWER_BOUND:
      value = (float)controller.get_lower_bound();
      enqueue_message(sender, msg_t::LOWER_BOUND_VALUE, (uint8_t *)&value, sizeof(value));
      break;
      
    case msg_t::LOWER_BOUND_VALUE:
      memcpy(&value, data, sizeof(value));
      Serial.printf("L %d %f\n", sender, value);
      break;

    case msg_t::SET_COST:
      memcpy(&value, data, sizeof(value));
      controller.set_cost((double)value);
      enqueue_message(sender, msg_t::ACK, nullptr, 0);
      break;

    case msg_t::GET_COST:
      value = (float)controller.get_cost();
      enqueue_message(sender, msg_t::COST_VALUE, (uint8_t *)&value, sizeof(value));
      break;
      
    case msg_t::COST_VALUE:
      memcpy(&value, data, sizeof(value));
      Serial.printf("c %d %f\n", sender, value);
      break;

    default:
      Serial.println("Couldn't decode message");
      break;
    }
    delete frm;
  }

  while (Serial.available() > 0)
  {
    char c = Serial.read();

    if (c == '\n')
    { // Found complete message
      buffer[buffer_pos] = '\0';
      interface(buffer);
      buffer_pos = 0;
    }
    else if (buffer_pos < BUFFER_SIZE - 1)
    {
      buffer[buffer_pos] = c;
      buffer_pos++;
    }
    else
    { // Buffer overflow - reset buffer position
      buffer_pos = 0;
    }
  }
}

void control_loop()
{
  static unsigned long last_run = micros();
  unsigned long current_run = micros();

  unsigned long h = (current_run - last_run) * 1e-3;
  if (h < sampInterval)
    return;

  int adc_value = analogRead(ADC_PIN);
  write_into_filter(adc_value);
  lux_value = adc2lux(read_from_filter());

  if (visualization)
  {
    Serial.print(lux_value);
    Serial.print(" ");
    Serial.print(r);
    Serial.println(" 0 50 ");
  }

  if (controller.get_feedback())
  {
    int u = (int)controller.get_control_signal(r, lux_value, h);
    analogWrite(LED_PIN, u);
    duty_cycle = (double)u / DAC_RANGE;
  }
  else
    duty_cycle = serial_duty_cycle;

  /* Performance Metrics/Others */
  iteration_counter++; // for averaging

  data = {lux_value, duty_cycle};
  last_minute_buffer.insert_new(data);

  // Energy
  energy += MAXIMUM_POWER * duty_cycle * DIFFERENCE(current_run, last_run) * pow(10, -6);

  // Visibility Error (not averaged)
  visibility_error += max(0, r - lux_value);

  // Flicker Error (not averaged)
  if (iteration_counter > 2)
  {
    if ((duty_cycle - prev_duty_cycle_1) * (prev_duty_cycle_1 - prev_duty_cycle_2) < 0)
      flicker_error += abs(duty_cycle - prev_duty_cycle_1) + abs(prev_duty_cycle_1 - prev_duty_cycle_2);
    prev_duty_cycle_2 = prev_duty_cycle_1;
    prev_duty_cycle_1 = duty_cycle;
  }
  else if (iteration_counter == 2)
    prev_duty_cycle_1 = duty_cycle;
  else
    prev_duty_cycle_2 = duty_cycle;

  if (stream_l)
    Serial.printf("s l %d %lf %d\n", LUMINAIRE, lux_value, millis());

  if (stream_d)
    Serial.printf("s d %d %lf %d\n", LUMINAIRE, duty_cycle, millis());

  if (stream_j)
    Serial.printf("s j %d %lf %d\n", LUMINAIRE, DIFFERENCE(DIFFERENCE(current_run, last_run), sampInterval * pow(10, 3)), millis());

  // Read last minute buffer
  if (buffer_read_counter < buffer_read_size)
  {
    int t = 20;

    if (buffer_read_size - buffer_read_counter < t)
      t = buffer_read_size - buffer_read_counter;

    for (int i = 0; i < t; i++)
    {
      data = last_minute_buffer.remove_oldest();

      if (buffer_l)
        Serial.printf("%f, ", data.lux_value);

      if (buffer_d)
        Serial.printf("%f, ", data.duty_cycle);

      buffer_read_counter++;
    }
  }
  else
  {
    if (buffer_d || buffer_l)
      Serial.println();
    buffer_d = 0;
    buffer_l = 0;
    buffer_read_size = 0;
    buffer_read_counter = 0;
  }

  last_run = current_run;
}

void enqueue_message(uint8_t recipient, msg_t type, uint8_t *message, std::size_t msg_size)
{
  if (type != msg_t::PING)
    Serial.printf("Enqueuing message of type %s with size %lu to recipient %d\n", message_type_translations[type], msg_size, recipient);
  can_frame *new_frame = new can_frame;
  std::size_t length = min(msg_size + 2, CAN_MAX_DLEN);
  new_frame->can_id = recipient;
  new_frame->can_dlc = length;
  if (msg_size > 0 && message != nullptr)
    memcpy(new_frame->data + 2, message, length - 2);
  new_frame->data[0] = LUMINAIRE;
  new_frame->data[1] = type;
  rp2040.fifo.push_nb((uint32_t)new_frame);
}

void master_calibrate_routine()
{
  luminaire_ids.clear();  
  luminaire_ids.push_back(LUMINAIRE);
  std::copy(other_luminaires.begin(), other_luminaires.end(), std::back_inserter(luminaire_ids));
  coupling_gains.clear();
  total_calibrations = 0;
  is_calibrating_as_master = true;
  is_calibrating = true;
  ready_luminaires.clear();
  calibration_stage = calibration_stage_t::WAIT_FOR_ACK;
  next_stage = calibration_stage_t::CALIBRATING;
  controller.set_feedback(false);
  analogWrite(LED_PIN, 0);
  for (const uint8_t luminaire : other_luminaires)
  {
    enqueue_message(luminaire, msg_t::OFF, nullptr, 0);
  }
}

void calibrate_loop()
{
  static unsigned long calibration_start_time = 0;
  static bool calibration_started = false;
  unsigned long current_time = millis();
  std::size_t i = 0;
  std::vector<double> recent_lux_readings;

  if (is_calibrating_as_master || is_calibrating)
  {
    switch (calibration_stage)
    {
    case calibration_stage_t::WAIT_FOR_ACK:
      if (ready_luminaires.size() == other_luminaires.size())
      {
        if (next_stage == calibration_stage_t::CALIBRATING)
          Serial.printf("Got ACKS from all luminaires, moving on to calibration (%d/%d)\n", coupling_gains.size() + 1, luminaire_ids.size() + 1);
        // Advance stage and send new messages
        calibration_stage = next_stage;
        calibration_start_time = current_time;
        calibration_started = false;
      }
      break;

    case calibration_stage_t::CALIBRATING:
      // Wait for capacitor discharge
      // FIXME - function that check signal stability may not work as requested
      // recent_lux_readings = last_minute_buffer.get_recent_lux_values(100);
      // if (!calibration_started && (current_time - calibration_start_time > CALIBRATION_START_DELAY || is_signal_stable(recent_lux_readings, 0.1)))
      if (!calibration_started && current_time - calibration_start_time > CALIBRATION_START_DELAY)
      {
        calibration_started = true;
        Serial.println("Comencing calibration now");
      }

      if (calibration_started)
      {
        int id_active = coupling_gains.size() == 0 ? -1 : luminaire_ids[coupling_gains.size() - 1];
        enqueue_message(BROADCAST, msg_t::READY_TO_READ, (uint8_t*) &id_active, sizeof(id_active));
        double lux_value = adc2lux(read_from_filter());
        if (coupling_gains.size() == 0)
          coupling_gains[id_active] = lux_value;
        else
          coupling_gains[id_active] = lux_value - coupling_gains[-1];

        // LED ON for the first gain, off after that
        if (coupling_gains.size() == 1)
          analogWrite(LED_PIN, 4095);
        else
          analogWrite(LED_PIN, 0);

        calibration_stage = calibration_stage_t::WAIT_FOR_ACK;
        next_stage = calibration_stage_t::CHANGE_STATE;
      }
      break;

    case calibration_stage_t::CHANGE_STATE:
      if (coupling_gains.size() == luminaire_ids.size() + 1)
      {
        enqueue_message(BROADCAST, msg_t::END, nullptr, 0);
        calibration_stage = calibration_stage_t::DONE;
        is_calibrating = false;
        is_calibrating_as_master = false;
        controller.set_feedback(true);
        print_map(coupling_gains);
      }
      else
      {
        calibration_stage = calibration_stage_t::WAIT_FOR_ACK;
        next_stage = calibration_stage_t::CALIBRATING;
        ready_luminaires.clear();
        for (i = 1; i < luminaire_ids.size(); i++)
        {
          // Turn on the right luminaire
          if (i == coupling_gains.size() - 1)
            enqueue_message(luminaire_ids[i], msg_t::ON, nullptr, 0);
          else
            enqueue_message(luminaire_ids[i], msg_t::OFF, nullptr, 0);
        }
      } 
      break;
    
    default:
      break;
    }
  }
}

void ping_loop()
{
  unsigned long current_time = millis();
  uint8_t ping_msg[4] = {ICC_WRITE_DATA, 0, 0, 0};
  if (current_time - time_since_last_ping > PING_TIMER)
  {
    time_since_last_ping = current_time;
    enqueue_message(0, msg_t::PING, nullptr, 0);
  }
}

void can_frame_to_bytes(can_frame *frm, uint8_t b[4])
{
  b[0] = frm->can_id;
  b[1] = frm->data[2];
  b[2] = frm->data[1];
  b[3] = frm->data[0];
}

void loop()
{
  control_loop();
  serial_command();
  ping_loop();
  calibrate_loop();
}

void process_can_frame(can_frame *frm)
{
  can_frame *new_frame = new can_frame;
  memcpy(new_frame, frm, sizeof(can_frame));
  rp2040.fifo.push_nb((uint32_t)new_frame);
}

void loop1()
{
  can_frame frm, *received_frm;

  uint8_t irq = can0.getInterrupts();
  if (irq & MCP2515::CANINTF_RX0IF)
  {
    can0.readMessage(MCP2515::RXB0, &frm);
    process_can_frame(&frm);
  }
  if (irq & MCP2515::CANINTF_RX1IF)
  {
    can0.readMessage(MCP2515::RXB1, &frm);
    process_can_frame(&frm);
  }

  if (rp2040.fifo.pop_nb((uint32_t *)&received_frm))
  {
    can0.sendMessage(received_frm);
    delete received_frm;
  }
}

void print_map(std::map<int, float> m)
{
  Serial.print("{");
  int i = 0;
  for (auto const &pair : m)
  {
    Serial.printf("%d: %f", pair.first, pair.second);
    i++;
    if (i < m.size())
      Serial.print(", ");
  }
  Serial.println("}");
}

bool is_signal_stable(const std::vector<double>& lux_values, double threshold) {
    if (lux_values.empty()) {
        return false;
    }
    
    std::vector<double> differences(lux_values.size());
    std::adjacent_difference(lux_values.begin(), lux_values.end(), differences.begin(), [](double a, double b) {
        return std::abs(a - b);
    });

    double average_difference = std::accumulate(differences.begin() + 1, differences.end(), 0.0) / (differences.size() - 1);
    return average_difference < threshold;
}