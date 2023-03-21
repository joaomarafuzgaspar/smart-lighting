#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

struct buffer_data {
  double lux_value;
  double duty_cycle;
};

template <int _size = 10>
class CircularBuffer {
public:
  CircularBuffer() : _head(0), _tail(0), _used(0){};

  int get_buffer_size() { return _size; };

  int get_used_space() { return _used; };

  bool is_empty() { return _used == 0; };

  bool is_full() { return _used == _size; };

  void insert_new(buffer_data elem) {
    if (!is_full())
      _used++;
    else {
      _tail++;
      if (_tail == _size)
        _tail = 0;
    }

    _head++;
    if (_head == _size)
        _head = 0;

    _buffer[_head] = elem;
  };

  buffer_data remove_oldest() {
    buffer_data oldest = _buffer[_tail];  // invalid if buffer empty

    if (!is_empty()) {
      _used--;
      _tail++;

      if (_tail == _size)
        _tail = 0;
    }

      return oldest;
  };

private:
  buffer_data _buffer[_size];
  int _head;
  int _tail;
  int _used;
};

#endif