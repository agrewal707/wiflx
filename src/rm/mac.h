/*
 * Copyright (C) 2021, Ajay Pal S. Grewal
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see https://opensource.org/licenses/GPL-2.0
 */

#ifndef WIFLX_RM_MAC_H
#define WIFLX_RM_MAC_H

#include <string>
#include <memory>
#include <queue>
#include <list>
#include <map>
#include <mutex>
#include <condition_variable>
#include <common/io_context.h>
#include <common/clock.h>
#include <common/timer.h>
#include <common/packet.h>
#include <common/txq.h>
#include <common/fq_codel.h>
#include <rm/iwf_mac_interface.h>
#include <rm/mac_phy_interface.h>
#include <rm/config.h>

namespace wiflx {
namespace rm {

class mac :
  public mac_phy::sap_user,
  public iwf_mac::sap_provider
{
public:
  mac (
    const config::mac &cfg,
    const wiflx::common::config::fq_codel &fq_codel_cfg,
    const wiflx::common::config::codel &codel_cfg);
  ~mac ();

  void set_mac_phy_sap_provider (mac_phy::sap_provider *p)
  {
    m_phy = p;
  }

  void set_iwf_mac_sap_user (iwf_mac::sap_user *u)
  {
    m_iwf = u;
  }

  void start ();
  void stop ();
  void timer_run ();

  //
  // IWF -> MAC
  //
  void send (const uint8_t tid, const uint32_t fid, std::string &&msdu);

  //
  // PHY -> MAC
  //
  virtual void on_receive (std::string &&mpdu, const stats &st);
  virtual void on_send ();

private:
  using packet = wiflx::common::packet;

  struct ra_cw
  {
    ra_cw ():
      m_min (0),
      m_max (0)
    {}

    ra_cw (uint32_t min, uint32_t max):
      m_min (min),
      m_max (max)
    {}

    uint32_t m_min;
    uint32_t m_max;
  };
  using ra_cw_map = std::unordered_map<uint8_t, ra_cw>;

  void enqueue_retry (packet &&p);
  void enqueue_ack (const common::messages::mac::ack &a);
  wiflx::common::packets  dequeue ();
  void release_packet (const common::messages::mac::ack &a);
  void release_all_packets ();

  void process_ack (
    const common::messages::mac::appdu &pdu,
    const common::clock::time_point &now);
  void process_data (
    const common::messages::mac::appdu &pdu,
    const common::clock::time_point &now);
  void process_poll (
    const common::messages::mac::appdu &pdu,
    const common::clock::time_point &now);

  bool fsm_is_ra_poll (const uint16_t rid);
  bool fsm_is_data_poll (const uint16_t rid);
  bool fsm_is_data_pending ();
  bool fsm_is_ra_backoff_done ();
  bool fsm_is_ra_ack (const common::messages::mac::ack &a);
  bool fsm_is_data_ack (const common::messages::mac::ack &a);
  void fsm_proc_ra_poll (const common::messages::mac::poll &p);
  void fsm_init_ra_backoff (const common::messages::mac::poll &p);
  void fsm_proc_ra_backoff (const common::messages::mac::poll &p);
  void fsm_send_ra ();
  void fsm_send_data ();
  void fsm_send_ack ();
  void fsm_wait ();
  void fsm_proc_data_timeout ();
  void fsm_proc_data_nak (const common::messages::mac::ack &a);
  void fsm_proc_data_ack (const common::messages::mac::ack &a);
  void fsm_release (bool from_ap);

  void reset_rlf_timer ();

  const config::mac &m_cfg;
  mac_phy::sap_provider *m_phy;
  iwf_mac::sap_user *m_iwf;
  uint16_t m_rid;
  std::shared_ptr<void> m_fsm;

  wiflx::common::fq_codel m_fq_codel;
  // queue to hold uplink unicast retry packets
  wiflx::common::txq_ctx m_retry_txq;
  // mutext to protect tx queues
  std::mutex m_txq_mutex;

  // pending ack for received frame
  std::list<common::messages::mac::ack> m_acks;
  std::mutex m_ack_mutex;

  uint8_t m_data_retries;
  int m_ra_backoff_counter;
  ra_cw_map m_ra_cw_map;
  uint16_t m_last_poll_seqid;
  uint16_t m_ra_seqid;
  uint16_t m_data_seqid;

  // timers are run in a separate thread
  wiflx::common::io_context m_timer_ioc;
  boost::asio::executor_work_guard<
    boost::asio::io_context::executor_type> m_timer_ioc_work;
  wiflx::common::timer m_ack_timer;
  wiflx::common::timer m_rlf_timer;

  bool m_stopped;

  friend class operation_;
};

} // namespace rm
} // namespace wiflx

#endif // WIFLX_RM_MAC_H
