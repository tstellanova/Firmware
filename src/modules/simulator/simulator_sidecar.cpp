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
#include <uORB/topics/battery_status.h>
#include <uORB/topics/timesync_status.h>
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


const int UORB_MSG_HEADER_LEN = 14;
const uint8_t UORB_MAGIC_V1 = 0xAA;


//forward declarations
void send_one_uorb_msg(Simulator::InternetProtocol via, const struct orb_metadata *meta,
    uint8_t* src, size_t len, int handle, uint8_t instance_id);
int subscribe_to_multi_topic(orb_id_t orb_msg_id, int instance_id, int interval);
uint16_t hash_from_msg_id(orb_id_t orb_msg_id);
void publish_uorb_msg_from_bytes(orb_id_t orb_msg_id, uint8_t instance_id, uint8_t* buf);




void update_px4_clock(uint64_t usec) {
  struct timespec ts = {};
  abstime_to_ts(&ts, usec);
  px4_clock_settime(CLOCK_MONOTONIC, &ts);
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


void Simulator::init()
{
  //create a map of all known uorb topics
  size_t count_orb_topics = orb_topics_count();

  const orb_metadata *const*all_topics = orb_get_topics();
  PX4_INFO("begin topic mapping %lu %p",count_orb_topics, all_topics);

  for (size_t i = 0; i < count_orb_topics; i++) {
    orb_id_t topic = all_topics[i];
    uint16_t hashval = hash_from_msg_id(topic);
    _uorb_hash_to_orb_id[hashval] = topic;
  }

  PX4_INFO("done with topic map");
  _fd = -1;
  _dest_sock_fd = -1;

  // subscribe to topics
  _actuator_outputs_sub = orb_subscribe_multi(ORB_ID(actuator_outputs), 0);
  _vehicle_status_sub =  subscribe_to_multi_topic(ORB_ID(vehicle_status), 0, 0);

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


void publish_uorb_msg_from_bytes(orb_id_t orb_msg_id, uint8_t instance_id, uint8_t* buf) {
  uint16_t hashval = hash_from_msg_id(orb_msg_id);
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
    //PX4_INFO("%s advert: %p", orb_msg_id->o_name, _uorb_hash_to_advert[key]);
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


  ssize_t prefix_byte_count = 0;
  uint64_t abstime_offset = 0;

  while (true) {
    // wait for new messages to arrive on socket
    int pret = ::poll(&fds[0], fd_count, 250);

    if (pret == 0) {
      // Timed out.
      //TODO temporary: force publish some attitude values
//      do_local_simulation();
      continue;
    }

    if (pret < 0) {
      PX4_WARN("poll error %d, %d", pret, errno);
      break;
    }

    if (fds[0].revents & POLLIN) {
      ssize_t avail_len = recvfrom(_dest_sock_fd,  &_recvbuf[prefix_byte_count], sizeof(_recvbuf) - prefix_byte_count , 0,
                         (struct sockaddr *) &_srcaddr, (socklen_t * ) & _addrlen);

      if ((avail_len > 0) && (prefix_byte_count > 0)) {
        avail_len += prefix_byte_count;
        //PX4_INFO("recovered: %ld",prefix_byte_count);
        prefix_byte_count = 0; //reset
      }
      uint8_t* offset_buf = &_recvbuf[0];
      uint32_t magic_search_count = 0;
      uint32_t msg_search_count = 0;
      while (avail_len > 0) {
        //find the first magic marker
        uint8_t magic = offset_buf[0];
        if ( magic != UORB_MAGIC_V1) {
          offset_buf += 1;
          avail_len -= 1;
          magic_search_count++;
          continue;
        }
        if (magic_search_count > 0) {
//          PX4_WARN("magic_search_count: %u",magic_search_count);
          magic_search_count = 0;
        }

        //verify we have at least header avail
        if (avail_len < UORB_MSG_HEADER_LEN) {
          //PX4_INFO("scraps");
          memcpy(&_recvbuf[0], offset_buf, avail_len);
          prefix_byte_count = avail_len;
          avail_len = 0;
          continue;
        }

        uint16_t hashval = (offset_buf[1] << 8) + offset_buf[2];
        uint64_t timestamp = ((uint64_t)offset_buf[3] << 56)
            + ((uint64_t)offset_buf[4] << 48)
            + ((uint64_t)offset_buf[5] << 40)
            + ((uint64_t)offset_buf[6] << 32)
            + ((uint64_t)offset_buf[7] << 24)
            + ((uint64_t)offset_buf[8] << 16)
            + ((uint64_t)offset_buf[9] << 8)
            + (uint64_t)offset_buf[10] ;


        uint8_t instance_id = offset_buf[11];
        uint16_t payload_len = (offset_buf[12] << 8) + offset_buf[13];

        // uint8_t magic
        // o_name hash uint16_t
        // timestamp uint64_t
        // instance id uint8_t
        // o_size uint16_t
        // payload

        orb_id_t orb_msg_id = _uorb_hash_to_orb_id[hashval];
        if (nullptr == orb_msg_id) {
          PX4_INFO("junk hash 0x%x count %u ", hashval, msg_search_count);
          offset_buf += 1;
          avail_len -= 1;
          msg_search_count++;
          continue;
        }
        else {
          msg_search_count = 0;
        }

        if (avail_len > payload_len) {
          //update the px4 clock on every remote uorb msg received
          update_px4_clock(timestamp + abstime_offset);
          if (0 == abstime_offset) {
            abstime_offset = timestamp;
          }

          //TODO There's a problem here in that the struct size is word-aligned, but input octet buf is not
          uint8_t* pBuf = (uint8_t*) &offset_buf[UORB_MSG_HEADER_LEN];
          publish_uorb_msg_from_bytes(orb_msg_id, instance_id, pBuf);

          offset_buf += (UORB_MSG_HEADER_LEN + payload_len);
          avail_len -= (UORB_MSG_HEADER_LEN + payload_len);
        }
        else {
          //PX4_WARN("avail: %ld need: %u", avail_len , payload_len);
          // move the remaining bytes to the front of the recvbuf, and read socket again
          memcpy(&_recvbuf[0], offset_buf, avail_len);
          prefix_byte_count = avail_len;
          avail_len = 0;
        }
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
  fds[0].fd = _actuator_outputs_sub;
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
    void* dest_payload_buf = (void*)&_sendbuf[UORB_MSG_HEADER_LEN];
    if (nullptr == src) {
      orb_copy(orb_msg_id, handle, dest_payload_buf);
    }
    else {
      memcpy(dest_payload_buf, src, len);
    }

    uint64_t timestamp =  hrt_absolute_time() + hrt_absolute_time_offset();
    uint16_t payload_len = orb_msg_id->o_size;
    uint16_t msg_len = UORB_MSG_HEADER_LEN + payload_len;
    //TODO store these in reverse id-to-hash map ?
    uint16_t hash_val = hash_from_msg_id(orb_msg_id);


//    PX4_INFO("encode %s[%d] (0x%x %d)", orb_msg_id->o_name, instance_id, hash_val, payload_len);
    // uorb wrapper header
    _sendbuf[0] = UORB_MAGIC_V1;
    _sendbuf[1] = (uint8_t)((hash_val >> 8) & 0xFF);
    _sendbuf[2] =  (uint8_t)(hash_val & 0xFF);

    //TODO send true timestamp as BigEndian uint64_t
    _sendbuf[3] = (uint8_t)((timestamp >> 56) & 0xFF);
    _sendbuf[4] = (uint8_t)((timestamp >> 48) & 0xFF);
    _sendbuf[5] = (uint8_t)((timestamp >> 40) & 0xFF);
    _sendbuf[6] = (uint8_t)((timestamp >> 32) & 0xFF);
    _sendbuf[7] = (uint8_t)((timestamp >> 24) & 0xFF);
    _sendbuf[8] = (uint8_t)((timestamp >> 16) & 0xFF);
    _sendbuf[9] = (uint8_t)((timestamp >> 8) & 0xFF);
    _sendbuf[10] = (uint8_t)(timestamp & 0xFF);

    _sendbuf[11] = instance_id;
    _sendbuf[12] = (uint8_t)((payload_len >> 8) & 0xFF);
    _sendbuf[13] = (uint8_t)(payload_len & 0xFF);

    // magic uint8_t
    // o_name hash uint16_t
    // timestamp uint64_t
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
      send_one_uorb_msg(_ip, std::get<0>(ctx), nullptr, 0, it->first, std::get<1>(ctx));
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

