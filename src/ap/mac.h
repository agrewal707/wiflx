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

#ifndef WIFLX_AP_MAC_H
#define WIFLX_AP_MAC_H

#include <string>
#include <memory>
#include <queue>
#include <map>
#include <unordered_map>
#include <list>
#include <mutex>
#include <common/io_context.h>
#include <common/clock.h>
#include <common/timer.h>
#include <common/txq.h>
#include <common/fq_codel.h>
#include <ap/iwf_mac_interface.h>
#include <ap/mac_phy_interface.h>
#include <ap/config.h>

namespace wiflx {
namespace ap {

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
  void send (const uint16_t rid, const uint8_t tid, const uint32_t fid, std::string &&msdu);

  //
  // PHY -> MAC
  //
  virtual void on_receive (std::string &&mpdu, const stats &st);
  virtual void on_send (const size_t bytes_sent);

private:
  //
  // remote context
  //
  struct rm_ctx
  {
    rm_ctx (uint16_t rid, int32_t airtime_quantum, wiflx::common::io_context &ioc);

    // not copyable
    rm_ctx (const rm_ctx &) = delete;
    rm_ctx& operator= (const rm_ctx&) = delete;

    // not movable
    rm_ctx (rm_ctx &&) = delete;
    rm_ctx& operator= (rm_ctx&&) = delete;

    ~rm_ctx ();

    uint16_t m_rid;
    bool m_active;
    wiflx::common::timer m_inactivity_timer;
    uint8_t m_poll_retries;
    uint8_t m_data_retries;
    uint16_t m_seqid;
    uint32_t m_bsr;
    int32_t m_deficit;
    std::mutex m_mutex;
  };

  //
  // packet
  //
  using packet = wiflx::common::packet;
  using packets = wiflx::common::packets;
  using pktsptr = std::shared_ptr<packets>;

  bool is_release (const packet&p) const
  {
    if (auto pd = p.m_wpd.lock ())
    {
      return !p;
    }
    return false;
  }

  void fsm_send_b_poll ();
  void fsm_send_u_poll (std::weak_ptr<rm_ctx> wrm);
  void fsm_send_b_data (std::weak_ptr<rm_ctx> wrm, pktsptr plp);
  void fsm_send_u_data (std::weak_ptr<rm_ctx> wrm, pktsptr plp);
  void send_u_ack_pdu (std::weak_ptr<rm_ctx> wrm);
  void send_pdu (std::weak_ptr<rm_ctx> wrm, pktsptr plp);
  void fsm_error ();
  void fsm_sched ();
  void fsm_p_ack_wait ();
  void fsm_d_ack_wait ();
  void fsm_ra_wait ();
  void fsm_p_ack_timeout ();
  void fsm_d_ack_timeout ();
  void poll_failure (std::weak_ptr<rm_ctx> wrm);
  void tx_failure (std::weak_ptr<rm_ctx> wrm);
  void tx_success (std::weak_ptr<rm_ctx> wrm, const common::messages::mac::ack &a);

  void process_ra (
    const common::messages::mac::rmpdu &pdu,
    const common::clock::time_point &now);
  void process_data (
    const common::messages::mac::rmpdu &pdu,
    const common::clock::time_point &now);
  void process_ack (
    const common::messages::mac::rmpdu &pdu,
    const common::clock::time_point &now);
  bool check_ack (const common::messages::mac::ack &a);

  bool admission_control_check (const common::messages::mac::rach &ra);
  void reset_inactivity_timer (std::shared_ptr<rm_ctx> rm);
  void remove_from_active_list (std::weak_ptr<rm_ctx> wrm);
  void schedule ();
  void schedule_retry ();
  std::pair<std::weak_ptr<rm_ctx>, pktsptr> get_next_remote ();

  void enqueue_retry (packet &&p);
  void enqueue_release (packet &&p);
  void enqueue_ack (const uint16_t rid, const common::messages::mac::ack &a);
  void release_packet (const common::messages::mac::ack &a);
  void release_all_packets ();
  void update_airtime (const uint16_t rid, const size_t size);

  static bool is_bcast_rid (const uint16_t rid)
  {
    return rid == common::messages::mac::BROADCAST_RID;
  }

  const config::mac &m_cfg;
  mac_phy::sap_provider *m_phy;
  iwf_mac::sap_user *m_iwf;
  std::shared_ptr<void> m_fsm;
  wiflx::common::fq_codel m_fq_codel;

  // rid -> remote context
  using rm_map = std::unordered_map<uint16_t, std::shared_ptr<rm_ctx>>;
  rm_map m_rm_map;
  std::mutex m_rm_map_mutex;
  std::list<std::weak_ptr<rm_ctx>> m_new_rms;
  std::list<std::weak_ptr<rm_ctx>> m_old_rms;
  std::mutex m_rm_list_mutex;

  // pending PDUs
  std::queue<std::pair<common::messages::mac::appdu,bool>> m_pdus;
  // queue to hold downlink unicast retry packets
  wiflx::common::txq_ctx m_retry_txq;
  // queue to hold downlink unicast release packets
  wiflx::common::txq_ctx m_release_txq;
  // mutex to protect retry and release queues
  std::mutex m_txq_mutex;

  // pending ack for received frame
  std::list<std::pair<uint16_t,common::messages::mac::ack>> m_acks;
  // mutex to protect ack queue
  std::mutex m_ack_mutex;

  // poll sequence id
  uint16_t m_poll_seqid;
  // current polled rid
  uint16_t m_poll_rid;
  // current broadcast poll airtime deficit
  int32_t m_b_poll_airtime_deficit;

  // timers are run in a separate thread
  wiflx::common::io_context m_timer_ioc;
  boost::asio::executor_work_guard<
    boost::asio::io_context::executor_type> m_timer_ioc_work;
  wiflx::common::timer m_ack_timer;

  bool m_stopped;

  static const uint8_t s_general_failure_timeout;

  friend class operation_;
  friend class ev_send;
};

} // namespace ap
} // namespace wiflx

#endif // WIFLX_AP_MAC_H
