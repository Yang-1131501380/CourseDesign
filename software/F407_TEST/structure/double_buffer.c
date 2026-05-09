#include "double_buffer.h"

void double_buffer_init(double_buffer_t* db, const raw_data_t* rd)
{
  db->size = rd->size / 2;
  db->buf[0] = (uint8_t*)rd->addr;
  db->buf[1] = db->buf[0] + db->size;
  db->active = 0;
  db->pending_valid = false;
  db->active_len = 0;
  db->pending_len = 0;
}

uint8_t* double_buffer_active(const double_buffer_t* db) { return db->buf[db->active]; }
uint8_t* double_buffer_pending(const double_buffer_t* db) { return db->buf[1 - db->active]; }
uint8_t* double_buffer_prepare_pending(double_buffer_t* db, size_t len)
{
  if ((db->pending_valid) || (len > db->size))
  {
    return NULL;
  }

  db->pending_len = len;
  return double_buffer_pending(db);
}
size_t double_buffer_size(const double_buffer_t* db) { return db->size; }
bool double_buffer_has_pending(const double_buffer_t* db) { return db->pending_valid; }

error_code_t double_buffer_fill_pending(double_buffer_t* db, const uint8_t* data, size_t len)
{
  if (db->pending_valid || len > db->size)
  {
    return ERR_FULL;
  }
  fast_copy(double_buffer_pending(db), data, len);
  db->pending_len = len;
  db->pending_valid = true;
  return ERR_OK;
}

error_code_t double_buffer_fill_active(double_buffer_t* db, const uint8_t* data, size_t len)
{
  if (len > db->size)
  {
    return ERR_FULL;
  }
  fast_copy(double_buffer_active(db), data, len);
  db->active_len = len;
  return ERR_OK;
}

void double_buffer_switch(double_buffer_t* db)
{
  if (db->pending_valid)
  {
    db->active ^= 1;
    db->active_len = db->pending_len;
    db->pending_valid = false;
  }
}

void double_buffer_enable_pending(double_buffer_t* db) { db->pending_valid = true; }
void double_buffer_commit_pending(double_buffer_t* db) { db->pending_valid = true; }
size_t double_buffer_pending_len(const double_buffer_t* db)
{
  return db->pending_valid ? db->pending_len : 0;
}
size_t double_buffer_active_len(const double_buffer_t* db) { return db->active_len; }
