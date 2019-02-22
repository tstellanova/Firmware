//
// Created by Todd Stellanova on 2019-02-01.
//


#include <termios.h>
#include <px4_log.h>
#include <px4_time.h>
#include <px4_tasks.h>
#include "simulator.h"
#include <simulator_config.h>
#include "errno.h"
#include <lib/ecl/geo/geo.h>
#include <drivers/drv_pwm_output.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <conversion/rotation.h>
#include <mathlib/mathlib.h>
#include <stdlib.h>

#include <uORB/uORB.h>
#include <uORB/uORBTopics.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
//#include <uORB/topics/sensor_combined.h>

//#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>


#include <limits>
#include <map>
#include <random>
#include <modules/uORB/uORB.h>

// tuple of orb ID hash and instance ID
typedef std::tuple<uint16_t,uint8_t> topic_instance_key;


static int _fd;
static int _dest_sock_fd;
static sockaddr_in _srcaddr;
static unsigned _addrlen = sizeof(_srcaddr);
static uint8_t _recvbuf[2048];
static uint8_t _sendbuf[2048];
static pthread_t _sender_thread;

static std::map<uint16_t, orb_id_t> _uorb_hash_to_orb_id;
static std::map<topic_instance_key, orb_advert_t> _uorb_hash_to_advert;


// track all of our uorb topic subscriptions
typedef std::tuple<orb_id_t,uint8_t> sub_handle_ctx;
static std::map<int, sub_handle_ctx> _uorb_sub_to_multi_orb_id;



static std::normal_distribution<float> _normal_distribution(0.0f, 3.0f);
static std::default_random_engine _noise_gen;


const int UORB_MSG_HEADER_LEN = 6;
const uint8_t UORB_MAGIC_V1 = 0xAA;


/// Some guesses as to accuracy of a fake accelerometer
const float ACCEL_ABS_ERR = 0.05f;
const float GYRO_ABS_ERR = 0.01f;
const float MAG_ABS_ERR  = 0.005f;
const float GPS_ABS_ERR = 1e-6f;

/// Fake home coordinates
const float HOME_LAT = 37.8f;
const float HOME_LON = -122.2f;
const float HOME_ALT = 500.0f;
const float HOME_MAG[] = { 22535E-5, 5384E-5, 42217-5 };


// Home magnetic declination
//from https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
//2019.10411,13.43814,61.24195,23169.0,48157.1,22534.6,5384.4,42217.4
//One gauss equals 1×10−4 T
// (nT / 1E9) * 1E4 = gauss
// (nT/1E5) = gauss




//forward declarations
void send_one_uorb_msg(Simulator::InternetProtocol via, const struct orb_metadata *meta,
    uint8_t* src, size_t len, int handle, uint8_t instance_id);
int subscribe_to_multi_topic(orb_id_t orb_msg_id, int instance_id, int interval);
uint16_t hash_from_msg_id(orb_id_t orb_msg_id);
void publish_uorb_msg(orb_id_t orb_msg_id, uint8_t instance_id, const void* buf);


/// Use unix time to simulate actual time
unsigned long get_os_clock_usec() {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  unsigned long time_in_micros = 1000000 * tv.tv_sec + tv.tv_usec;
  return time_in_micros;
}

void update_px4_clock(unsigned long usec) {
  struct timespec ts = {};
  abstime_to_ts(&ts, usec);
  px4_clock_settime(CLOCK_MONOTONIC, &ts);
  //TODO use abstime_to_ts instead?
}

static hrt_abstime _simulated_clock_usec = 0;

void set_simulated_clock(hrt_abstime usec) {
  _simulated_clock_usec = usec;
}

hrt_abstime get_simulated_clock_usec() {
  return _simulated_clock_usec;
}

void increment_simulation_clock(uint32_t usec) {
  _simulated_clock_usec += usec;
}

const double STD_PRESS = 101325.0;  // static pressure at sea level (Pa)
const double  STD_TEMP = 288.15;    // standard temperature at sea level (K)
const double LAPSE_RATE = -0.0065;   // standard temp altitude lapse rate (K/m)
const double MOL_MASS  = 0.0289644;  // molar mass of Earth's air (kg/mol)
const double GAS_CONSTANT_R  = 8.31432;    // universal gas constant, R
const double ACCEL_ONE_G = 9.80665f;	//gravity acceleration (m/s^2)


/// Convert altitude (meters) to standard barometric pressure (Pascals)
/// Note: this formula is likely only useful under 10k feet
float altitude_to_baro_pressure(float alt)  {
  double big_alt = (double)alt;
  double base = STD_TEMP / (STD_TEMP + (LAPSE_RATE * big_alt));
  double exp = (ACCEL_ONE_G * MOL_MASS) / (GAS_CONSTANT_R * LAPSE_RATE);
  double val = STD_PRESS * pow(base, exp);
  return (float)val;
}



uint16_t hash_from_msg_id(orb_id_t orb_msg_id) {
  int namelen = strlen(orb_msg_id->o_name);
  uint16_t hash_val = crc_calculate((const uint8_t*) orb_msg_id->o_name, namelen);
  return  hash_val;
}



int subscribe_to_multi_topic(orb_id_t orb_msg_id, int instance_id, int interval) {
  int handle =  orb_subscribe_multi(orb_msg_id, instance_id);
  if (handle >= 0) {
    PX4_INFO("subd %s [%0d]", orb_msg_id->o_name,  instance_id);
    if (interval > 0) {
      orb_set_interval(handle, interval);
    }

    _uorb_sub_to_multi_orb_id[handle] = std::make_tuple(orb_msg_id, instance_id);
  }
  else {
    PX4_ERR("orb_subscribe_multi %s failed (%i)", orb_msg_id->o_name, errno);
  }

  return handle;
}

void init_simulated_clock() {
  hrt_abstime start_time = get_os_clock_usec();
  start_time = (start_time / 1000) * 1000;
  set_simulated_clock(start_time);
  update_px4_clock(start_time);

  PX4_WARN("Simulator::init with %llu %llu", start_time, hrt_absolute_time());

}

void Simulator::init()
{
  //create a map of all known uorb topics
  size_t count_orb_topics = orb_topics_count();
//  const struct orb_metadata *all_topics = reinterpret_cast<const orb_metadata *>(orb_get_topics());

  const orb_metadata *const*all_topics = orb_get_topics();

  PX4_INFO("begin topic mapping %lu %p",count_orb_topics, all_topics);

  for (size_t i = 0; i < count_orb_topics; i++) {
    orb_id_t topic = all_topics[i];
    uint16_t hashval = hash_from_msg_id(topic);
    _uorb_hash_to_orb_id[hashval] = topic;
  }

  PX4_INFO("done with topic map");

  //TODO temp -- normally we'd update the clock based on eg HIL_SENSOR
  init_simulated_clock();

  _fd = -1;
  _dest_sock_fd = -1;

  // subscribe to topics
  for (unsigned i = 0; i < (sizeof(_actuator_outputs_sub) / sizeof(_actuator_outputs_sub[0])); i++) {
    _actuator_outputs_sub[i] =  subscribe_to_multi_topic(ORB_ID(actuator_outputs), i, 0);
  }
  _vehicle_status_sub =  subscribe_to_multi_topic(ORB_ID(vehicle_status), 0, 0);

//  subscribe_to_multi_topic(ORB_ID(vehicle_local_position_setpoint), 0, 0);
//  subscribe_to_multi_topic(ORB_ID(trajectory_setpoint), 0, 0);

  this->_initialized = true;
}


void start_thread(pthread_t *thread, void *(*start_routine) (void *)) {
  pthread_attr_t thread_attr;
  pthread_attr_init(&thread_attr);
  pthread_attr_setstacksize(&thread_attr, PX4_STACK_ADJUSTED(4000));

  struct sched_param param;
  (void)pthread_attr_getschedparam(&thread_attr, &param);

  param.sched_priority = SCHED_PRIORITY_DEFAULT;
  (void)pthread_attr_setschedparam(&thread_attr, &param);

  pthread_create(thread, &thread_attr, start_routine, nullptr);
  pthread_attr_destroy(&thread_attr);
}

void Simulator::start_sender() {
  start_thread(&_sender_thread, Simulator::sending_trampoline);
}



bool Simulator::init_connection() {
  struct sockaddr_in _myaddr{};
  _myaddr.sin_family = AF_INET;
  _myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  _myaddr.sin_port = htons(_port);

  if (_ip == InternetProtocol::UDP) {
    if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      PX4_ERR("Creating UDP socket failed: %s", strerror(errno));
      return false;
    }

    if (bind(_fd, (struct sockaddr *) &_myaddr, sizeof(_myaddr)) < 0) {
      PX4_ERR("bind for UDP port %i failed (%i)", _port, errno);
      return false;
    }

    PX4_INFO("Waiting for client to connect on UDP port %u", _port);

    while (true) {
      // receiving a first packet means we're ready to chat
      int len = recvfrom(_fd, _recvbuf, sizeof(_recvbuf), 0,
                         (struct sockaddr *) &_srcaddr, (socklen_t * ) & _addrlen);

      if (len > 0) {
        _dest_sock_fd = _fd;
        break;
      }
      else {
        //give some time for receive buffer to settle
        system_sleep(1);
      }
    }

    PX4_INFO("Client connected on UDP port %u.", _port);
  }
  else {
    PX4_INFO("Waiting for client to connect on TCP port %u", _port);

    while (true) {
      if ((_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        PX4_ERR("Creating TCP socket failed: %s", strerror(errno));
        return  false;
      }

      int enable = 1;
      int ret = setsockopt(_fd, IPPROTO_TCP, TCP_NODELAY, (char*) &enable, sizeof(enable));
      if (ret != 0) {
        PX4_ERR("setsockopt failed: %s", strerror(errno));
      }

      if (bind(_fd, (struct sockaddr *) &_myaddr, sizeof(_myaddr)) < 0) {
        PX4_ERR("ERROR on binding");
        return false;
      }

      //start listening for clients
      struct  sockaddr_in cli_addr = {};
      listen(_fd,5);
      unsigned int clilen = sizeof(cli_addr);

      // Accept actual connection from the client
      int newsockfd = accept(_fd, (struct sockaddr *)&cli_addr, &clilen);

      if (ret == 0) {
        _dest_sock_fd = newsockfd;
        break;
      }
      else {
        PX4_ERR("accept err %d",ret);
        close(_fd);
        system_sleep(1);
      }
    }

    PX4_INFO("Client connected on TCP port %u.", _port);
  }

  return true;
}


float get_noisy_value(float val, float err) {
  return (val + (err * _normal_distribution(_noise_gen)) );
}

void send_fast_cadence_fake_sensors(Simulator::InternetProtocol via) {

  unsigned long common_time =  hrt_absolute_time();
  if (common_time == 0) {
    PX4_WARN("bogus hrt");
    return;
  }

  sensor_gyro_s gyro_report = {
    .timestamp = common_time,
    .device_id = 2293768,
    .integral_dt = 5 * 1000000,
    .x_integral = 1.0f,
    .y_integral = 1.0f,
    .z_integral = 1.0f,

    .x = get_noisy_value(0.001f, GYRO_ABS_ERR),
    .y = get_noisy_value(0.001f, GYRO_ABS_ERR),
    .z = get_noisy_value(0.001f, GYRO_ABS_ERR),

    .x_raw = (int16_t)(gyro_report.x * 1E3),
    .y_raw = (int16_t)(gyro_report.y * 1E3),
    .z_raw = (int16_t)(gyro_report.z * 1E3),

    .temperature = get_noisy_value(25.0f, 0.01f),
  };
  publish_uorb_msg(ORB_ID(sensor_gyro),0, (const void*) &gyro_report);
//  PX4_WARN("gyro x: %f", (double) gyro_report.x);

  float xacc = get_noisy_value(0.0, ACCEL_ABS_ERR);
  float yacc = get_noisy_value(0.0, ACCEL_ABS_ERR);
  float zacc = get_noisy_value(ACCEL_ONE_G, ACCEL_ABS_ERR);

  sensor_accel_s accel_report = {
      .timestamp = common_time ,
      .device_id = 1376264,
      .x_raw = (int16_t)(xacc / (ACCEL_ONE_G / 1E3)),
      .y_raw = (int16_t)(yacc / (ACCEL_ONE_G / 1E3)),
      .z_raw = (int16_t)(zacc / (ACCEL_ONE_G / 1E3)),
      .x = xacc,
      .y = yacc,
      .z = zacc,

      .temperature = get_noisy_value(25.0f, 0.01f),
  };
  publish_uorb_msg(ORB_ID(sensor_accel),0, (const void*) &accel_report);


  float xmag = get_noisy_value(HOME_MAG[0], MAG_ABS_ERR);
  float ymag = get_noisy_value(HOME_MAG[1], MAG_ABS_ERR);
  float zmag = get_noisy_value(HOME_MAG[2], MAG_ABS_ERR);

  mag_report mag_report = {
      .timestamp = common_time ,
      .device_id = 196616,
      .x_raw = (int16_t)(xmag * 1000.0f),
      .y_raw = (int16_t)(ymag * 1000.0f),
      .z_raw = (int16_t)(zmag * 1000.0f),
      .x = xmag,
      .y = ymag,
      .z = zmag,

      .temperature = get_noisy_value(25.0f, 0.01f),
  };
  publish_uorb_msg(ORB_ID(sensor_mag),0, (const void*) &mag_report);


  // In order for sensor fusion to align,
  // baro pressure needs to match the altitude given by GPS
  float shared_alt = get_noisy_value(HOME_ALT, 0.1);
  float abs_pressure =  altitude_to_baro_pressure(shared_alt);

  sensor_baro_s baro_report = {
      .timestamp = common_time,
      .device_id = 478459,
      .pressure = abs_pressure,
      .temperature = 25.0f,
  };
  publish_uorb_msg(ORB_ID(sensor_baro),0, (const void*) &baro_report);


}

void send_fake_gps_msgs(Simulator::InternetProtocol via) {
  int32_t common_alt = (int32_t)(1E3* get_noisy_value(HOME_ALT, 0.1));
  unsigned long common_time =  hrt_absolute_time();

  vehicle_gps_position_s gps_report = {
      .timestamp = common_time,
      .time_utc_usec = get_simulated_clock_usec(),
      .lat = (int32_t)(1E7* get_noisy_value(HOME_LAT, GPS_ABS_ERR)),
      .lon = (int32_t)(1E7* get_noisy_value(HOME_LON, GPS_ABS_ERR)),
      .alt = common_alt,
      .alt_ellipsoid = common_alt,
      .eph = 1.0f,
      .epv = 2.0f,
      .vel_m_s = get_noisy_value(0.0, 0.025f),
      .vel_n_m_s = get_noisy_value(0.0, 0.025f),
      .vel_e_m_s = get_noisy_value(0.0, 0.025f),
      .vel_d_m_s = get_noisy_value(0.0, 0.025f),
      .cog_rad = get_noisy_value(0.0f, 0.001f),
      .vel_ned_valid = true,
      .fix_type = 3,
      .satellites_used = 10,
  };
  send_one_uorb_msg(via, ORB_ID(vehicle_gps_position), (uint8_t*)&gps_report, sizeof(gps_report), 0, 0);

//  vehicle_global_position_s hil_global_pos = {
//      .timestamp = gps_report.timestamp,
//      .lat = gps_report.lat / ((double)1e7),
//      .lon = gps_report.lon / ((double)1e7),
//      .alt = gps_report.alt / 1000.0f,
//      .vel_n = gps_report.vel_n_m_s / 100.0f,
//      .vel_e = gps_report.vel_e_m_s / 100.0f,
//      .vel_d = gps_report.vel_d_m_s / 100.0f,
//      .eph = gps_report.eph,
//      .epv = gps_report.epv,
//  };
//  send_one_uorb_msg(via, ORB_ID(vehicle_global_position), (uint8_t*)&hil_global_pos, sizeof(hil_global_pos), 0, 0);

}

void send_slow_cadence_fake_sensors(Simulator::InternetProtocol via) {

  unsigned long common_time = hrt_absolute_time();

//  send_fake_gps_msgs(via);

//  system_power_s system_power = {
//    .timestamp =  common_time,
//    .voltage5v_v = 5.0,
//    .voltage3v3_v = 3.3,
//    .v3v3_valid = 1,
//    .usb_connected = 0,
//    .brick_valid = 1,
//    .usb_valid = 0,
//    .servo_valid = 1,
//    .periph_5v_oc = 0,
//    .hipower_5v_oc = 0
//  };
//  send_one_uorb_msg(via, ORB_ID(system_power), (uint8_t*)&system_power, sizeof(system_power), 0, 0);


  battery_status_s batt_report =  {
    .timestamp =  common_time,
    .voltage_v = 16.0,
    .cell_count = 4,
    .connected = true,
    .system_source = true,
//    .warning = battery_status_s::BATTERY_WARNING_CRITICAL,
  };
  send_one_uorb_msg(via, ORB_ID(battery_status), (uint8_t*)&batt_report, sizeof(batt_report), 0, 0);

}


#define SIMULATOR_TIME_RATIO 1

void do_local_simulation(Simulator::InternetProtocol via) {
  static hrt_abstime _last_realtime_clock = 0;
  static hrt_abstime _last_fast_cadence_send = 0;
//  static hrt_abstime _last_slow_cadence_send = 0;


  hrt_abstime real_time = get_os_clock_usec();
  hrt_abstime delta_time = (real_time - _last_realtime_clock);
//  PX4_WARN("delta_time: %llu", delta_time);
  _last_realtime_clock = real_time;

  for (int i = 0; i < SIMULATOR_TIME_RATIO; i++) {
    increment_simulation_clock(delta_time);

    update_px4_clock(get_simulated_clock_usec());
    hrt_abstime local_elapsed_usec = hrt_absolute_time();

    if ((local_elapsed_usec - _last_fast_cadence_send) > 125) {
      send_fast_cadence_fake_sensors(via);
      _last_fast_cadence_send = local_elapsed_usec;

//      if ((local_elapsed_usec - _last_slow_cadence_send) > 1000000) {
//        send_slow_cadence_fake_sensors(via);
//        _last_slow_cadence_send = local_elapsed_usec;
//      }
    }
  }

}



void publish_uorb_msg(orb_id_t orb_msg_id, uint8_t instance_id, const void* buf) {
  uint16_t hashval = hash_from_msg_id( orb_msg_id);
  std::tuple<uint16_t,uint8_t> key = std::make_tuple(hashval, instance_id);
  orb_advert_t advert = _uorb_hash_to_advert[key];

  int up_instance_id = instance_id;

  int ret = orb_publish_auto(
      orb_msg_id,
      &advert,
      buf,
      &up_instance_id,
      ORB_PRIO_HIGH);

  if (_uorb_hash_to_advert[key] != advert) {
    _uorb_hash_to_advert[key] = advert;
    PX4_INFO("%s advert: %p", orb_msg_id->o_name, _uorb_hash_to_advert[key]);
  }

  if (OK != ret) {
    PX4_ERR("publish err: %d", ret);
  }
  else {
    PX4_DEBUG("pub: %s [%d]", orb_msg_id->o_name, instance_id);
  }
}



void Simulator::recv_loop() {

  struct pollfd fds[2];
  memset(fds, 0, sizeof(fds));
  unsigned fd_count = 1;
  fds[0].fd = _dest_sock_fd;
  fds[0].events = POLLIN;

  PX4_WARN("Wait to recv msgs from partner...");

  while (true) {

    //TODO temporary: force publish some attitude values
    do_local_simulation(_ip);

    // wait for new messages to arrive on socket
    int pret = ::poll(&fds[0], fd_count, 250);
    if (pret == 0) {
      // Timed out.
      continue;
    }

    if (pret < 0) {
      PX4_WARN("poll error %d, %d", pret, errno);
      break;
    }

    if (fds[0].revents & POLLIN) {
      ssize_t avail_len = recvfrom(_dest_sock_fd,  _recvbuf, sizeof(_recvbuf), 0,
                         (struct sockaddr *) &_srcaddr, (socklen_t * ) & _addrlen);

      uint8_t* offset_buf = &_recvbuf[0];
      while (avail_len > 0) {
        //find the first magic marker
        uint8_t magic = offset_buf[0];
        if ( magic != UORB_MAGIC_V1) {
          offset_buf += 1;
          avail_len -= 1;
          continue;
        }

        uint16_t hashval = (offset_buf[1] << 8) + offset_buf[2];
        uint8_t instance_id = offset_buf[3];
        uint16_t payload_len = (offset_buf[4] << 8) + offset_buf[5];

        // uint8_t magic
        // o_name hash uint16_t
        // instance id uint8_t
        // o_size uint16_t
        // payload

        //PX4_INFO("rcvd 0x%x %d %d", hashval, instance_id, payload_len);
        //std::tuple<uint16_t,uint8_t> key = std::make_tuple(hashval, instance_id);
        //_uorb_tuple_to_orb_id[key];

        orb_id_t orb_msg_id = _uorb_hash_to_orb_id[hashval];
        if (nullptr == orb_msg_id) {
          PX4_INFO("junk hash 0x%x instance %u ", hashval, instance_id);
          offset_buf += 1;
          avail_len -= 1;
          continue;
        }

        publish_uorb_msg(orb_msg_id, instance_id, (const uint8_t*) &offset_buf[UORB_MSG_HEADER_LEN]);

        offset_buf += (UORB_MSG_HEADER_LEN + payload_len);
        avail_len -= (UORB_MSG_HEADER_LEN + payload_len);
      }
    }
  }

}


void *Simulator::sending_trampoline(void * /*unused*/)
{
  _instance->send_loop();
  return nullptr;
}



void Simulator::send_loop()
{
#ifdef __PX4_DARWIN
  pthread_setname_np("sim_send");
#else
  pthread_setname_np(pthread_self(), "sim_send");
#endif


  //wait for connection to be available
  while (_dest_sock_fd <= 0) {
    PX4_WARN("wait for conn");
    system_sleep(5);
  }

  PX4_WARN("Simulator::send_loop");

  px4_pollfd_struct_t fds[1] = {};
  fds[0].fd = _actuator_outputs_sub[0];
  fds[0].events = POLLIN;

  while (true) {
    // Wait for up to 250ms for sentinel uorb topic to update
    int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 250);
    if (pret == 0) {
      // Timed out, try again.
      continue;
    }

    if (pret < 0) {
      PX4_ERR("poll error %s", strerror(errno));
      break;
    }

    if (fds[0].revents & POLLIN) {
      // Our sentinel topic updated: check all subscribed topics
      poll_topics();
    }
  }
}



void send_one_uorb_msg(
    Simulator::InternetProtocol via,
    orb_id_t orb_msg_id,
    uint8_t* src,
    size_t len,
    int handle,
    uint8_t instance_id) {
  //only bother processing uorb messages if we have a connected partner
  if (_dest_sock_fd > 0) {
    if (nullptr == src) {
      orb_copy(orb_msg_id, handle, (void *) &_sendbuf[UORB_MSG_HEADER_LEN]);
    }
    else {
      memcpy((void *) &_sendbuf[UORB_MSG_HEADER_LEN], src, len);
    }

    uint16_t payload_len = orb_msg_id->o_size;
    uint16_t msg_len = UORB_MSG_HEADER_LEN + payload_len;
    //TODO store these in reverse id-to-hash map
    uint16_t hash_val = hash_from_msg_id(orb_msg_id);


//    PX4_INFO("encode %s[%d] (0x%x %d)", orb_msg_id->o_name, instance_id, hash_val, payload_len);
    // uorb wrapper header
    _sendbuf[0] = UORB_MAGIC_V1;
    _sendbuf[1] = (hash_val >> 8) & 0xFF;
    _sendbuf[2] = hash_val & 0xFF;
    _sendbuf[3] = instance_id;
    _sendbuf[4] = (payload_len >> 8) & 0xFF;
    _sendbuf[5] = payload_len & 0xFF;

    // magic uint8_t
    // o_name hash uint16_t
    // instance id uint8_t (for multi instance)
    // o_size (length of payload) uint16_t
    // payload

    ssize_t sent_len;

    if (via == Simulator::InternetProtocol::UDP) {
      sent_len = ::sendto(_dest_sock_fd, _sendbuf, msg_len, 0, (struct sockaddr *)&_srcaddr, sizeof(_srcaddr));
    }
    else {
      sent_len = ::send(_dest_sock_fd, _sendbuf, msg_len, 0);
    }

    if (sent_len > 0) {
      PX4_DEBUG("sent %s 0x%x %u %u",orb_msg_id->o_name, hash_val, instance_id, payload_len);
    }
  }

}

void Simulator::poll_topics()
{
  //check all subscribed topics for updates

  std::map<int, sub_handle_ctx>::iterator it;
  for (it = _uorb_sub_to_multi_orb_id.begin(); it != _uorb_sub_to_multi_orb_id.end(); it++) {
    bool updated = false;
    orb_check(it->first,  &updated);
    if (updated) {
      sub_handle_ctx ctx = it->second;
      send_one_uorb_msg(_ip, std::get<0>(ctx), nullptr, 0, updated, std::get<1>(ctx));
    }
  }

}



void Simulator::runloop() {

  if (init_connection()) {
    start_sender();
    recv_loop();
  }

  PX4_WARN("Simulator::runloop exit");
}

