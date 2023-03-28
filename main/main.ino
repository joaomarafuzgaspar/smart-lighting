#include <sstream>
#include <set>
#include <hardware/flash.h>
#include "mcp2515.h"
#include "quickselect.h"
#include "controller.h"
#include "circular_buffer.h"


#define DIFFERENCE(a, b) ((a) > (b) ? (a) - (b):(b) - (a))

// Constants
const int DAC_RANGE = 4096;
const int DAC_BITS = 12;
const int LED_PIN = 15;
const int ADC_PIN = A0;
const unsigned long PING_TIMER = 5000;
const double Vcc = 3.3;
const double MAXIMUM_POWER = 2.65 * 5.8e-3;
const int sampInterval = 10;

const int R = 10e3;
const double b = 6.1521825181113625;
const double m = -0.8;

const int MEDIAN_FILTER_SIZE = 5;
int last_measurements[MEDIAN_FILTER_SIZE] = {1, 1, 1, 1, 1};

// Luminaire ID
uint8_t LUMINAIRE;

enum inter_core_cmds {
  //From core1 to core0: contains data read (16 bit)
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
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};
std::set<uint8_t> other_luminaires;
unsigned long time_since_last_ping = 0;

double adc2resistance(int adc_value)
{
  /* V_ADC = adc_value / DAC_RANGE * VCC; R_LDR = VCC / V_ADC * R - R */
  return R * (DAC_RANGE / ((double) adc_value) - 1);
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
double calibrate_gain() {
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

void setup() {
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

void setup1() {
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

void serial_command() {
  const int BUFFER_SIZE = 128;
  static char buffer[BUFFER_SIZE];
  static int buffer_pos = 0;

  uint8_t bytes[4];  
  while (rp2040.fifo.available()) {
    rp2040.fifo.pop_nb((uint32_t*) bytes);
    Serial.print("Received msg: ");
    Serial.println(*((uint8_t*) bytes));
    if (bytes[0] == 0) {
      Serial.print("Got ping message from ");
      Serial.print(bytes[1]);
      if (other_luminaires.insert(bytes[1]).second)
        Serial.println(", inserted as new");
      else
        Serial.println();
    }
  }
  
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') { // Found complete message
      buffer[buffer_pos] = '\0';
      interface(buffer);
      buffer_pos = 0;
    }
    else if (buffer_pos < BUFFER_SIZE - 1) {
      buffer[buffer_pos] = c;
      buffer_pos++;
    } 
    else { // Buffer overflow - reset buffer position
      buffer_pos = 0;
      }
  }
}

void loop() {
  static unsigned long last_run = micros();
  unsigned long current_run = micros();

  unsigned long h = (current_run - last_run) * 1e-3;
  if (h < sampInterval)
    return;

  serial_command();

  int adc_value = analogRead(ADC_PIN);
  write_into_filter(adc_value);
  lux_value = adc2lux(read_from_filter());

  if (visualization) {
    Serial.print(lux_value);
    Serial.print(" ");
    Serial.print(r);
    Serial.println(" 0 50 ");
  }

  if (controller.get_feedback()) {
    int u = (int) controller.get_control_signal(r, lux_value, h);
    analogWrite(LED_PIN, u);
    duty_cycle = (double) u / DAC_RANGE;
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
  if (iteration_counter > 2) {
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
  if (buffer_read_counter < buffer_read_size) {
    int t = 20;

    if (buffer_read_size - buffer_read_counter < t)
      t = buffer_read_size - buffer_read_counter;

    for (int i = 0; i < t; i++) {
      data = last_minute_buffer.remove_oldest();

      if (buffer_l)
        Serial.printf("%f, ", data.lux_value);
      
      if (buffer_d)
        Serial.printf("%f, ", data.duty_cycle);

      buffer_read_counter++;
    }
  }
  else {
    if (buffer_d || buffer_l)
        Serial.println();
    buffer_d = 0;
    buffer_l = 0;
    buffer_read_size = 0;
    buffer_read_counter = 0;
  }

  last_run = current_run;

  unsigned long current_time = millis();
  uint8_t ping_msg[4] = {ICC_WRITE_DATA, 0, 0, 0};
  if (current_time - time_since_last_ping > PING_TIMER) {
    time_since_last_ping = current_time;
    rp2040.fifo.push_nb(*((uint32_t*) ping_msg));
  }
}

void can_frame_to_bytes(can_frame *frm, uint8_t b[4]) {
  b[0] = frm->can_id; b[1] = frm->data[2];
  b[2] = frm->data[1]; b[3] = frm->data[0];
}

void loop1() {
  can_frame frm;
  uint8_t bytes[4];

  uint8_t irq = can0.getInterrupts();
  if (irq & MCP2515::CANINTF_RX0IF) {
    can0.readMessage(MCP2515::RXB0, &frm);
    can_frame_to_bytes(&frm, bytes);
    rp2040.fifo.push_nb(*((uint32_t*) bytes));
  }
  if (irq & MCP2515::CANINTF_RX1IF) {
    can0.readMessage(MCP2515::RXB1, &frm);
    can_frame_to_bytes(&frm, bytes);
    rp2040.fifo.push_nb(*((uint32_t*) bytes));
  }  

  if (rp2040.fifo.pop_nb((uint32_t*)bytes)) {
    if (bytes[0] == ICC_WRITE_DATA) {
      frm.can_id = bytes[1];
      frm.can_dlc = 3;
      frm.data[0] = bytes[2];
      frm.data[1] = bytes[3];
      frm.data[2] = LUMINAIRE;
      can0.sendMessage(&frm);
    }
  }

}
