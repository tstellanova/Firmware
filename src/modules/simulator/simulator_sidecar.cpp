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


#include <limits>
#include <map>
#include <random>
#include <modules/uORB/uORB.h>


static int _fd;
static int _dest_sock_fd;
static sockaddr_in _srcaddr;
static unsigned _addrlen = sizeof(_srcaddr);
static uint8_t _recvbuf[2048];
static uint8_t _sendbuf[2048];
static pthread_t _sender_thread;

static std::map<uint16_t, orb_id_t> _uorb_hash_to_orb_id;
static std::map<uint16_t, orb_advert_t> _uorb_hash_to_advert;

static std::map<int, orb_id_t> _uorb_sub_to_orb_id;
static std::map<orb_id_t, int> _orb_id_to_sub_handle;


static std::normal_distribution<float> _normal_distribution(-1.0f, 1.0f);
static std::default_random_engine _noise_gen;


const int UORB_MSG_HEADER_LEN = 5;


/// Some guesses as to accuracy of a fake accelerometer
const float ACCEL_ABS_ERR = 1e-2f;
const float GYRO_ABS_ERR = 1e-2f;
const float MAG_ABS_ERR  = 1e-2f;
const float GPS_ABS_ERR = 1e-3f;

/// Fake home coordinates
const float HOME_LAT = 37.8f;
const float HOME_LON = -122.2f;
const float HOME_ALT = 500.0f;


//forward declarations
void send_one_uorb_msg(Simulator::InternetProtocol via, const struct orb_metadata *meta,
    uint8_t* src, size_t len, int handle, uint8_t instance_id);
int subscribe_to_multi_topic(orb_id_t orb_msg_id, int idx, int interval);
void prep_for_topic_transactions(orb_id_t orb_msg_id);
uint16_t hash_from_msg_id(orb_id_t orb_msg_id);


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

/**
 * Prepare to send or receive serialized uorb messages with the given ID
 *
 * @param orb_msg_id
 */
void prep_for_topic_transactions(orb_id_t orb_msg_id) {
  uint16_t hashval = hash_from_msg_id(orb_msg_id);
  // the map of encoded hashval to orb_id
  _uorb_hash_to_orb_id[hashval] = orb_msg_id;
  //create a map slot for the advert, but don't advertise yet (unless we want to publish)
  _uorb_hash_to_advert[hashval] = nullptr;
}

int subscribe_to_multi_topic(orb_id_t orb_msg_id, int idx, int interval) {

  int handle =  orb_subscribe_multi(orb_msg_id, idx);
  if (handle >= 0) {
    PX4_INFO("subd %s [%0d]", orb_msg_id->o_name,  idx);
    if (interval > 0) {
      orb_set_interval(handle, interval);
    }

    _uorb_sub_to_orb_id[handle] = orb_msg_id;
    _orb_id_to_sub_handle[orb_msg_id] = handle;

    //be prepared to send or receive these from remote partner
    prep_for_topic_transactions(orb_msg_id);
  }
  else {
    PX4_ERR("orb_subscribe_multi %s failed (%i)", orb_msg_id->o_name, errno);
  }

  return handle;
}


void Simulator::init()
{
  PX4_WARN("Simulator::init");
  _fd = -1;
  _dest_sock_fd = -1;

  // subscribe to topics
  for (unsigned i = 0; i < (sizeof(_actuator_outputs_sub) / sizeof(_actuator_outputs_sub[0])); i++) {
    _actuator_outputs_sub[i] =  subscribe_to_multi_topic(ORB_ID(actuator_outputs), i, 0);
  }
  _vehicle_status_sub =  subscribe_to_multi_topic(ORB_ID(vehicle_status), 0, 0);

  subscribe_to_multi_topic(ORB_ID(vehicle_local_position_setpoint), 0, 0);
  subscribe_to_multi_topic(ORB_ID(trajectory_setpoint), 0, 0);

  this->_initialized = true;
}


void Simulator::start_sender() {

  pthread_attr_t sender_thread_attr;
  pthread_attr_init(&sender_thread_attr);
  pthread_attr_setstacksize(&sender_thread_attr, PX4_STACK_ADJUSTED(4000));

  struct sched_param param;
  (void)pthread_attr_getschedparam(&sender_thread_attr, &param);

  param.sched_priority = SCHED_PRIORITY_DEFAULT;
  (void)pthread_attr_setschedparam(&sender_thread_attr, &param);

  pthread_create(&_sender_thread, &sender_thread_attr, Simulator::sending_trampoline, nullptr);
  pthread_attr_destroy(&sender_thread_attr);

}

void Simulator::init_connection() {
  struct sockaddr_in _myaddr{};
  _myaddr.sin_family = AF_INET;
  _myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  _myaddr.sin_port = htons(_port);

  if (_ip == InternetProtocol::UDP) {
    if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      PX4_ERR("Creating UDP socket failed: %s", strerror(errno));
      return;
    }

    if (bind(_fd, (struct sockaddr *) &_myaddr, sizeof(_myaddr)) < 0) {
      PX4_ERR("bind for UDP port %i failed (%i)", _port, errno);
      return;
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
        return;
      }

      int enable = 1;
      int ret = setsockopt(_fd, IPPROTO_TCP, TCP_NODELAY, (char*) &enable, sizeof(enable));

      if (ret != 0) {
        PX4_ERR("setsockopt failed: %s", strerror(errno));
      }

      if (bind(_fd, (struct sockaddr *) &_myaddr, sizeof(_myaddr)) < 0) {
        PX4_ERR("ERROR on binding");
        return;
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

}


float range_random(float min, float max)
{
  float s = rand() / (float)RAND_MAX;
  return (min + s * (max - min));
}

float get_noisy_value(float val, float err) {
  return (val + err * _normal_distribution(_noise_gen));
}

void send_fast_cadence_fake_sensors(Simulator::InternetProtocol via) {

  sensor_gyro_s gyro_report = {
    .timestamp = hrt_absolute_time(),
    .device_id = 2293768,

    .x = get_noisy_value(0.001f, GYRO_ABS_ERR),
    .y = get_noisy_value(0.001f, GYRO_ABS_ERR),
    .z = get_noisy_value(0.001f, GYRO_ABS_ERR),

    .x_raw = (int16_t)(gyro_report.x  * 1000.0f),
    .y_raw = (int16_t)(gyro_report.y * 1000.0f),
    .z_raw = (int16_t)(gyro_report.z * 1000.0f),

    .temperature = get_noisy_value(25.0f, 0.01f),
  };
  send_one_uorb_msg(via, ORB_ID(sensor_gyro), (uint8_t*)&gyro_report, sizeof(gyro_report), 0, 0);


  float xacc = get_noisy_value(0.001, ACCEL_ABS_ERR);
  float yacc = get_noisy_value(0.001, ACCEL_ABS_ERR);
  float zacc = get_noisy_value(ACCEL_ONE_G, ACCEL_ABS_ERR);

  sensor_accel_s accel_report = {
      .timestamp = hrt_absolute_time(),
      .device_id = 1376264,
      .x_raw = (int16_t)(xacc / (ACCEL_ONE_G / 1000.0f)),
      .y_raw = (int16_t)(yacc / (ACCEL_ONE_G / 1000.0f)),
      .z_raw = (int16_t)(zacc / (ACCEL_ONE_G / 1000.0f)),
      .x = xacc,
      .y = yacc,
      .z = zacc,

      .temperature = 25.0f,
  };
  send_one_uorb_msg(via, ORB_ID(sensor_accel), (uint8_t*)&accel_report, sizeof(accel_report), 0, 0);


  float xmag = get_noisy_value(0.001,  MAG_ABS_ERR);
  float ymag = get_noisy_value(0.001, MAG_ABS_ERR);
  float zmag = get_noisy_value(0.001, MAG_ABS_ERR);

  mag_report mag_report = {
      .timestamp = hrt_absolute_time(),
      .device_id = 196616,
      .x_raw = (int16_t)(xmag * 1000.0f),
      .y_raw = (int16_t)(ymag * 1000.0f),
      .z_raw = (int16_t)(zmag * 1000.0f),
      .x = xmag,
      .y = ymag,
      .z = zmag,

      .temperature = 25.0f,
  };
//  int mag_instance_id = 0;
//  uint16_t  hashval = hash_from_msg_id(ORB_ID(sensor_mag));
//  orb_advert_t advert = _uorb_hash_to_advert[hashval];
//  if (nullptr != advert) {
//    orb_publish_auto(ORB_ID(sensor_mag), &advert, (uint8_t *) &mag_report, &mag_instance_id,
//                     ORB_PRIO_DEFAULT);
//  }
//  else {
    send_one_uorb_msg(via, ORB_ID(sensor_mag), (uint8_t *) &mag_report, sizeof(mag_report), 0, 0);
//  }

  // In order for sensor fusion to align,
  // baro pressure needs to match the altitude given by GPS
  float shared_alt = get_noisy_value(HOME_ALT, 0.1);
  float abs_pressure =  altitude_to_baro_pressure(shared_alt);

  sensor_baro_s baro_report = {
    .timestamp = hrt_absolute_time(),
    .device_id = 478459,
    .pressure = abs_pressure,
    .temperature = 25.0f,
  };
  send_one_uorb_msg(via, ORB_ID(sensor_baro), (uint8_t*)&baro_report, sizeof(baro_report), 0, 0);


}

void send_slow_cadence_fake_sensors(Simulator::InternetProtocol via) {

  vehicle_gps_position_s gps_report = {
      .timestamp = hrt_absolute_time(),
      .lat = (int32_t)(1E7* get_noisy_value(HOME_LAT, GPS_ABS_ERR)),
      .lon = (int32_t)(1E7* get_noisy_value(HOME_LON, GPS_ABS_ERR)),
      .alt = (int32_t)(1E3* get_noisy_value(HOME_ALT, 0.1)),
      .eph = 1e-2f,
      .epv = 1e-2f,
      .vel_m_s = get_noisy_value(0.01, 0.25f),
      .vel_n_m_s = get_noisy_value(0.01, 0.25f),
      .vel_e_m_s = get_noisy_value(0.01, 0.25f),
      .vel_d_m_s = get_noisy_value(0.01, 0.25f),
      .cog_rad = get_noisy_value(0.01f, 0.1f),
      .fix_type = 3,
      .satellites_used = 10,
  };
  send_one_uorb_msg(via, ORB_ID(vehicle_gps_position), (uint8_t*)&gps_report, sizeof(gps_report), 0, 0);

  //  system_power_s system_power = {
//    .timestamp =  hrt_absolute_time(),
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
    .timestamp =  hrt_absolute_time(),
    .voltage_v = 16.0,
    .cell_count = 4,
    .connected = true,
    .system_source = true,
//    .warning = battery_status_s::BATTERY_WARNING_CRITICAL,
  };
  send_one_uorb_msg(via, ORB_ID(battery_status), (uint8_t*)&batt_report, sizeof(batt_report), 0, 0);

}


void Simulator::recv_loop() {

  // these are values we expect to be sent from our remote partner
  prep_for_topic_transactions(ORB_ID(sensor_gyro));
  prep_for_topic_transactions(ORB_ID(sensor_accel));
  prep_for_topic_transactions(ORB_ID(sensor_mag));
  prep_for_topic_transactions(ORB_ID(sensor_baro));
  prep_for_topic_transactions(ORB_ID(vehicle_gps_position));

  prep_for_topic_transactions(ORB_ID(system_power));
  prep_for_topic_transactions(ORB_ID(battery_status));


  init_connection();

  struct pollfd fds[2];
  memset(fds, 0, sizeof(fds));
  unsigned fd_count = 1;
  fds[0].fd = _dest_sock_fd;
  fds[0].events = POLLIN;

  while (true) {
    // wait for new messages to arrive
    int pret = ::poll(&fds[0], fd_count, 1000);
    if (pret == 0) {
      // Timed out.
      //TODO temporary: force publish some attitude values

      for (int i = 0; i < 5; i++) {
        send_fast_cadence_fake_sensors(_ip);
      }
      send_slow_cadence_fake_sensors(_ip);

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
        uint16_t hashval = (offset_buf[0] << 8) + offset_buf[1];
        int instance_id = offset_buf[2];
        uint16_t payload_len = (offset_buf[3] << 8) + offset_buf[4];

        // o_name hash uint16_t
        // instance id uint8_t
        // o_size uint16_t
        // payload

//        PX4_INFO("rcvd 0x%x %d %d", hashval, instance_id, payload_len);
        orb_id_t orb_msg_id = _uorb_hash_to_orb_id[hashval];
        orb_advert_t advert = _uorb_hash_to_advert[hashval];

        if (nullptr == orb_msg_id) {
          PX4_INFO("hash 0x%x msg_id %p advert %p", hashval, orb_msg_id, advert);
          offset_buf += 1;
          avail_len -= 1;
          continue;
        }

        if (ORB_ID(actuator_outputs) != orb_msg_id) {
          int ret = orb_publish_auto(
              orb_msg_id,
              &advert,
              (const void *) &offset_buf[UORB_MSG_HEADER_LEN],
              &instance_id,
              ORB_PRIO_HIGH);

          if (_uorb_hash_to_advert[hashval] != advert) {
            _uorb_hash_to_advert[hashval] = advert;
            PX4_INFO("new %s advert: %p", orb_msg_id->o_name, _uorb_hash_to_advert[hashval]);
          }

          if (OK != ret) {
            PX4_ERR("publish err: %d", ret);
          }
          else {
            PX4_DEBUG("pub: %s [%d]", orb_msg_id->o_name, instance_id);
          }
        }

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

  PX4_WARN("start Simulator::send_loop");

  px4_pollfd_struct_t fds[1] = {};
  fds[0].fd = _actuator_outputs_sub[0];
  fds[0].events = POLLIN;

  while (true) {
    // Wait for up to 100ms for data.
    int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);
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
    uint16_t hash_val = hash_from_msg_id(orb_msg_id); //TODO store these in reverse id-to-hash map


//    PX4_WARN("encode %s[%d] (0x%x %d)", orb_msg_id->o_name, instance_id, hash_val, payload_len);
    // uorb wrapper header
    _sendbuf[0] = (hash_val >> 8) & 0xFF;
    _sendbuf[1] = hash_val & 0xFF;
    _sendbuf[2] = instance_id;
    _sendbuf[3] = (payload_len >> 8) & 0xFF;
    _sendbuf[4] = payload_len & 0xFF;

    // o_name hash uint16_t
    // instance id uint8_t (for multi instance)
    // o_size (size of payload) uint16_t
    // payload

    ssize_t sent_len;

    if (via == Simulator::InternetProtocol::UDP) {
      sent_len = ::sendto(_dest_sock_fd, _sendbuf, msg_len, 0, (struct sockaddr *)&_srcaddr, sizeof(_srcaddr));
    }
    else {
      sent_len = ::send(_dest_sock_fd, _sendbuf, msg_len, 0);
    }

    if (sent_len > 0) {
      PX4_DEBUG("sent %s 0x%x %d %d",orb_msg_id->o_name, hash_val, instance_id, payload_len);
    }
  }

}

void Simulator::poll_topics()
{
  //check all subscribed topics for updates
  std::map<int, orb_id_t>::iterator it;

  for ( it = _uorb_sub_to_orb_id.begin(); it != _uorb_sub_to_orb_id.end(); it++ ) {
    bool updated = false;
    orb_check(it->first, &updated);
    if (updated) {
      send_one_uorb_msg(_ip, it->second, nullptr, 0, it->first, 0); ///TODO not right instance?
    }
  }

}



void Simulator::runloop() {
  start_sender();

  recv_loop();

  PX4_WARN("Simulator::runloop exit");
}

