#pragma once

#include "c_base.h"

typedef struct
{
  uint8_t* buf[2];
  size_t size;
  int active;
  bool pending_valid;
  size_t active_len;
  size_t pending_len;
} double_buffer_t;

void double_buffer_init(double_buffer_t* db, const raw_data_t* rd);
uint8_t* double_buffer_active(const double_buffer_t* db);
uint8_t* double_buffer_pending(const double_buffer_t* db);
uint8_t* double_buffer_prepare_pending(double_buffer_t* db, size_t len);
size_t double_buffer_size(const double_buffer_t* db);
bool double_buffer_has_pending(const double_buffer_t* db);
error_code_t double_buffer_fill_pending(double_buffer_t* db, const uint8_t* data, size_t len);
error_code_t double_buffer_fill_active(double_buffer_t* db, const uint8_t* data, size_t len);
void double_buffer_switch(double_buffer_t* db);
void double_buffer_enable_pending(double_buffer_t* db);
void double_buffer_commit_pending(double_buffer_t* db);
size_t double_buffer_pending_len(const double_buffer_t* db);
size_t double_buffer_active_len(const double_buffer_t* db);
