#pragma once

struct Vector3{
  double x;
  double y;
  double yaw;
};

static constexpr Vector3 sz_to_rz[] = {
  #include <path/TR/RedField/sz_to_rz_r.csv>
};
static constexpr int sz_to_rz_size = sizeof(sz_to_rz) / sizeof(Vector3);

static constexpr Vector3 rz_to_ts1[] = {
  #include <path/TR/RedField/rz_to_ts1_r.csv>
};
static constexpr int rz_to_ts1_size = sizeof(rz_to_ts1) / sizeof(Vector3);

static constexpr Vector3 ts1_to_rz[] = {
  #include <path/TR/RedField/ts1_to_rz_r.csv>
};
static constexpr int ts1_to_rz_size = sizeof(ts1_to_rz) / sizeof(Vector3);


static constexpr Vector3 rz_to_ts2[] = {
  #include <path/TR/RedField/rz_to_ts2_r.csv>
};
static constexpr int rz_to_ts2_size = sizeof(rz_to_ts2) / sizeof(Vector3);

static constexpr Vector3 ts2_to_rz[] = {
  #include <path/TR/RedField/ts2_to_rz_r.csv>
};
static constexpr int ts2_to_rz_size = sizeof(ts2_to_rz) / sizeof(Vector3);


static constexpr Vector3 rz_to_ts3[] = {
  #include <path/TR/RedField/rz_to_ts3_r.csv>
};
static constexpr int rz_to_ts3_size = sizeof(rz_to_ts3) / sizeof(Vector3);

static constexpr Vector3 ts3_to_rz[] = {
  #include <path/TR/RedField/ts3_to_rz_r.csv>
};
static constexpr int ts3_to_rz_size = sizeof(ts3_to_rz) / sizeof(Vector3);


static constexpr Vector3 rz_to_ts4[] = {
  #include <path/TR/RedField/rz_to_ts4_r.csv>
};
static constexpr int rz_to_ts4_size = sizeof(rz_to_ts4) / sizeof(Vector3);

static constexpr Vector3 ts4_to_rz[] = {
  #include <path/TR/RedField/ts4_to_rz_r.csv>
};
static constexpr int ts4_to_rz_size = sizeof(ts4_to_rz) / sizeof(Vector3);


static constexpr Vector3 rz_to_ts5[] = {
  #include <path/TR/RedField/rz_to_ts5_r.csv>
};
static constexpr int rz_to_ts5_size = sizeof(rz_to_ts5) / sizeof(Vector3);

static constexpr Vector3 ts5_to_rz[] = {
  #include <path/TR/RedField/ts5_to_rz_r.csv>
};
static constexpr int ts5_to_rz_size = sizeof(ts5_to_rz) / sizeof(Vector3);