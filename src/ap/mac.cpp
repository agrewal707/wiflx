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

#include <ap/mac.h>

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>
#include <boost/msm/front/euml/state_grammar.hpp>
#include <common/log.h>
#include <common/error.h>
#include <common/profiling.h>

namespace wiflx {
namespace ap {

using std::mutex;
using std::unique_lock;
using std::lock_guard;
using std::weak_ptr;
namespace msm = boost::msm;
namespace mpl = boost::mpl;
using namespace msm::front;
// for And_ operator
using namespace msm::front::euml;

const uint8_t mac::s_general_failure_timeout = 100; // msecs

//
// mac::rm_ctx
//
mac::rm_ctx::rm_ctx (uint16_t rid, int32_t airtime_quantum, wiflx::common::io_context &ioc):
  m_rid (rid),
  m_active (false),
  m_inactivity_timer (ioc),
  m_poll_retries (0),
  m_data_retries (0),
  m_seqid (0),
  m_deficit (airtime_quantum)
{
  WIFLX_LOG_FUNCTION (this);
}

mac::rm_ctx::~rm_ctx ()
{
  WIFLX_LOG_FUNCTION (this);
}

//
// MAC operation FSM
//
struct ev_send
{
  ev_send (weak_ptr<mac::rm_ctx> wrm, mac::pktsptr plp) :
    m_wrm (wrm),
    m_plp (plp),
    m_u_ack (false)
  {}

  ev_send (bool u_ack) :
    m_u_ack (u_ack)
  {}

  weak_ptr<mac::rm_ctx> m_wrm;
  mac::pktsptr m_plp;
  bool m_u_ack;
};
struct ev_error {};
struct ev_sent {};
struct ev_rcvd {};
struct ev_timeout {};

struct operation_ :
  public msm::front::state_machine_def<operation_>
{
  operation_ (mac *m);
  ~operation_ ();

  // Required when using 'Defer' in FSM
  typedef int activate_deferred_events;

  // FSM states
  #define DECLARE_ST( __state_name__ ) \
  struct __state_name__ : public msm::front::state<> \
  { \
    std::string name () const { \
      return #__state_name__; \
    }\
  };

  DECLARE_ST (st_idle);
  DECLARE_ST (st_sending_b_poll);
  DECLARE_ST (st_sending_u_poll);
  DECLARE_ST (st_sending_b_data);
  DECLARE_ST (st_sending_u_data);
  DECLARE_ST (st_sending_u_ack);
  DECLARE_ST (st_d_ack_wait);
  DECLARE_ST (st_p_ack_wait);
  DECLARE_ST (st_ra_wait);

  // initial state
  typedef st_idle initial_state;

  template <class Fsm, class Event>
  void no_transition (Event const &ev, Fsm &fsm, int state)
  {
    WIFLX_LOG_ERROR("ev: {}, st: {}", typeid(ev).name(), state);
  }

  template <class Fsm, class Event>
  void exception_caught (Event const &ev, Fsm &fsm, std::exception &ex)
  {
    WIFLX_LOG_ERROR("ev: {}, ex: {}", typeid(ev).name(), ex.what());
  }

  //
  // Guards
  //
  struct is_b_poll
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    bool operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_TRACE_L1("GD: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      if (auto rm = ev.m_wrm.lock())
        return mac::is_bcast_rid(rm->m_rid) && ev.m_plp && ev.m_plp->empty();
      else
        return true;
    }
  };
  struct is_u_poll
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    bool operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_TRACE_L1("GD: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      if (auto rm = ev.m_wrm.lock())
        return !mac::is_bcast_rid(rm->m_rid) && ev.m_plp && ev.m_plp->empty();
      else
        return false;
    }
  };
  struct is_b_data
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    bool operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_TRACE_L1("GD: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      if (auto rm = ev.m_wrm.lock())
        return mac::is_bcast_rid(rm->m_rid) && ev.m_plp && !ev.m_plp->empty();
      else
        return false;
    }
  };
  struct is_u_data
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    bool operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_TRACE_L1("GD: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      if (auto rm = ev.m_wrm.lock())
        return !mac::is_bcast_rid(rm->m_rid) && ev.m_plp && !ev.m_plp->empty();
      else
        return false;
    }
  };
  struct is_u_ack
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    bool operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_TRACE_L1("GD: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      return ev.m_u_ack;
    }
  };
  //
  // Transitions
  //
  struct tr_send_b_poll
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src,TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_send_b_poll ();
    }
  };
  struct tr_send_u_poll
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src,TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_send_u_poll (ev.m_wrm);
    }
  };
  struct tr_send_b_data
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src,TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_send_b_data (ev.m_wrm, ev.m_plp);
    }
  };
  struct tr_send_u_data
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src,TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_send_u_data (ev.m_wrm, ev.m_plp);
    }
  };
  struct tr_send_u_ack
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src,TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());
    }
  };
  struct tr_error
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src,TargetState &dst)
    {
      WIFLX_LOG_ERROR("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_error ();
    }
  };
  struct tr_sched
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src,TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_sched ();
    }
  };
  struct tr_ra_wait
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src,TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_ra_wait ();
    }
  };
  struct tr_p_ack_wait
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src,TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_p_ack_wait ();
    }
  };
  struct tr_d_ack_wait
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src,TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_d_ack_wait ();
    }
  };
  struct tr_p_ack_timeout
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src,TargetState &dst)
    {
      WIFLX_LOG_WARNING("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_p_ack_timeout ();
    }
  };
  struct tr_d_ack_timeout
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src,TargetState &dst)
    {
      WIFLX_LOG_WARNING("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_d_ack_timeout ();
    }
  };
  // Transition table for MAC operation
  struct transition_table : mpl::vector<
    //    Start               Event                 Next              Action               Guard
    //  +--------------------+-------------------- +------------------+-------------------+-------+
    Row < st_idle,            ev_send,            st_sending_b_poll,    tr_send_b_poll,    is_b_poll >,
    Row < st_idle,            ev_send,            st_sending_u_poll,    tr_send_u_poll,    is_u_poll >,
    Row < st_idle,            ev_send,            st_sending_b_data,    tr_send_b_data,    is_b_data >,
    Row < st_idle,            ev_send,            st_sending_u_data,    tr_send_u_data,    is_u_data >,
    Row < st_idle,            ev_send,            st_sending_u_ack,     tr_send_u_ack,     is_u_ack >,
    Row < st_idle,            ev_error,           st_idle,              tr_error,          none >,

    Row < st_sending_b_poll,  ev_sent,            st_ra_wait,           tr_ra_wait,        none >,
    Row < st_sending_u_poll,  ev_sent,            st_p_ack_wait,        tr_p_ack_wait,     none >,
    Row < st_sending_b_data,  ev_sent,            st_idle,              tr_sched,          none >,
    Row < st_sending_u_data,  ev_sent,            st_d_ack_wait,        tr_d_ack_wait,     none >,
    Row < st_sending_u_ack,   ev_sent,            st_idle,              none,              none >,
    Row < st_sending_u_ack,   ev_send,            st_sending_u_ack,     Defer,             none >,

    Row < st_p_ack_wait,      ev_timeout,         st_idle,              tr_p_ack_timeout,  none >,
    Row < st_p_ack_wait,      ev_rcvd,            st_idle,              tr_sched,          none >,

    Row < st_d_ack_wait,      ev_timeout,         st_idle,              tr_d_ack_timeout,  none >,
    Row < st_d_ack_wait,      ev_rcvd,            st_idle,              tr_sched,          none >,

    Row < st_ra_wait,         ev_timeout,         st_idle,              tr_sched,          none >,
    Row < st_ra_wait,         ev_rcvd,            st_idle,              tr_sched,          none >
  > {};

  mac *m_mac;
};

//
// FSM front end
//
operation_::operation_ (mac *m):
  m_mac (m)
{
  WIFLX_LOG_FUNCTION (this);
}

operation_::~operation_ ()
{
  WIFLX_LOG_FUNCTION (this);
}

// FSM backend
using operation = msm::back::state_machine<operation_>;
// FSM helpers
#define M_FSM (std::static_pointer_cast<operation>(m_fsm))
#define FSM_PROCESS_EVENT(x) M_FSM->process_event(x)

//
// MAC
//
mac::mac (
  const config::mac &cfg,
  const wiflx::common::config::fq_codel &fq_codel_cfg,
  const wiflx::common::config::codel &codel_cfg) :
  m_cfg (cfg),
  m_phy (nullptr),
  m_iwf (nullptr),
  m_fsm (std::make_shared<operation> (this)),
  m_fq_codel (fq_codel_cfg, codel_cfg),
  m_poll_seqid (0),
  m_poll_rid (0),
  m_b_poll_airtime_deficit (m_cfg.b_poll_airtime_quantum),
  m_timer_ioc (),
  m_timer_ioc_work (boost::asio::make_work_guard(m_timer_ioc)),
  m_ack_timer (m_timer_ioc),
  m_stopped (true)
{
  WIFLX_LOG_FUNCTION (this);

  // add broadcast rm context
  auto rm = std::make_shared<rm_ctx> (common::messages::mac::BROADCAST_RID, m_cfg.rm_airtime_quantum, m_timer_ioc);
  const auto result = m_rm_map.emplace (common::messages::mac::BROADCAST_RID, rm);
  if (!result.second)
    throw std::runtime_error ("duplicate rid");
}

mac::~mac ()
{
  WIFLX_LOG_FUNCTION (this);
}

void mac::start ()
{
  WIFLX_LOG_FUNCTION (this);

  if (m_stopped)
  {
    m_stopped = false;
    M_FSM->start();
    schedule ();
  }
}

void mac::stop ()
{
  WIFLX_LOG_FUNCTION (this);

  if (!m_stopped)
  {
    m_stopped = true;
    m_acks.clear ();
    m_ack_timer.cancel ();
    m_timer_ioc_work.reset();
  }
}

//
// FSM
//
void mac::fsm_send_b_poll ()
{
  WIFLX_LOG_FUNCTION (this);

  m_poll_rid = common::messages::mac::BROADCAST_RID;

  common::messages::mac::appdu pdu;
  pdu.set_rid (common::messages::mac::BROADCAST_RID);
  auto *p = pdu.mutable_p ();
  p->set_seqid (++m_poll_seqid);
  // TODO - set network load factor to enable remotes
  // to adjust the RA contention window.
  // p->set_load_factor (get_load_factor ());
  WIFLX_LOG_DEBUG ("TX POLL -- rid:{:d}, seq:{:d}", pdu.rid(), p->seqid());

  std::string psdu;
  if (pdu.SerializeToString (&psdu))
    m_phy->send (std::move(psdu));
  else
    FSM_PROCESS_EVENT(ev_error());
}

void mac::fsm_send_u_poll (weak_ptr<rm_ctx> wrm)
{
  WIFLX_LOG_FUNCTION (this);

  common::messages::mac::appdu pdu;

  if (auto rm = wrm.lock())
  {
    m_poll_rid = rm->m_rid;

    // ACK
    {
      lock_guard<mutex> lock (m_ack_mutex);
      while (!m_acks.empty())
      {
        auto &pair = m_acks.front ();
        WIFLX_LOG_DEBUG ("TX ACK -- rid:{:d}, seq:{:d}", pair.first, pair.second.seqid());
        pdu.set_rid (pair.first);
        auto *a = pdu.add_a ();
        a->Swap (&pair.second);
        m_acks.pop_front ();
      }
    }

    // POLL
    {
      pdu.set_rid (rm->m_rid);
      auto *p = pdu.mutable_p ();
      p->set_seqid (++m_poll_seqid);
      WIFLX_LOG_DEBUG ("TX POLL -- rid:{:d}, seq:{:d}", pdu.rid(), p->seqid());
    }

    std::string psdu;
    if (pdu.SerializeToString (&psdu))
    {
      update_airtime (pdu.rid(), psdu.size());
      m_phy->send (std::move(psdu));
    }
    else
      FSM_PROCESS_EVENT(ev_error());
  }
}

void mac::fsm_send_b_data (weak_ptr<rm_ctx> wrm, pktsptr plp)
{
  WIFLX_LOG_FUNCTION (this);

  common::messages::mac::appdu pdu;

  if (auto rm = wrm.lock())
  {
    m_poll_rid = 0;

    // DATA
    {
      for (auto &pkt : *plp)
      {
        pdu.set_rid (rm->m_rid);
        auto *d = pdu.add_d ();
        d->set_seqid (pkt.m_seqid);
        auto *payload = d->mutable_payload ();
        *payload = pkt.m_msdu;

        // broadcast packets are not retried

        WIFLX_LOG_DEBUG ("RM {:d} DEFICIT {:d}", rm->m_rid, rm->m_deficit);

        WIFLX_LOG_DEBUG ("TX DATA -- rid:{:d}, seq:{:d}, size: {:d}",
          pdu.rid(), d->seqid(), d->payload().size());
      }
    }

    std::string psdu;
    if (pdu.SerializeToString (&psdu))
    {
      update_airtime (pdu.rid(), psdu.size());
      m_phy->send (std::move(psdu));
    }
    else
      FSM_PROCESS_EVENT(ev_error());
  }
}

void mac::fsm_send_u_data (weak_ptr<rm_ctx> wrm, pktsptr plp)
{
  WIFLX_LOG_FUNCTION (this);

  common::messages::mac::appdu pdu;

  if (auto rm = wrm.lock())
  {
    m_poll_rid = rm->m_rid;

    // ACK
    {
      lock_guard<mutex> lock (m_ack_mutex);
      while (!m_acks.empty())
      {
        auto &pair = m_acks.front ();
        WIFLX_LOG_DEBUG ("TX ACK -- rid:{:d}, seq:{:d}", pair.first, pair.second.seqid());
        pdu.set_rid (pair.first);
        auto *a = pdu.add_a ();
        a->Swap (&pair.second);
        m_acks.pop_front ();
      }
    }

    // DATA
    bool null_frame = false;
    {
      for (auto &pkt : *plp)
      {
        pdu.set_rid (rm->m_rid);
        auto *d = pdu.add_d ();
        d->set_seqid (pkt.m_seqid);
        auto *payload = d->mutable_payload ();
        *payload = pkt.m_msdu;

        enqueue_retry (std::move (pkt));

        WIFLX_LOG_DEBUG ("RM {:d} DEFICIT {:d}", rm->m_rid, rm->m_deficit);

        WIFLX_LOG_DEBUG ("TX DATA -- rid:{:d}, seq:{:d}, size: {:d}",
          pdu.rid(), d->seqid(), d->payload().size());

        null_frame = d->payload().size() == 0;
      }
    }

    // POLL
    if (!null_frame)
    {
      pdu.set_rid (rm->m_rid);
      auto *p = pdu.mutable_p ();
      p->set_seqid (++m_poll_seqid);
      WIFLX_LOG_DEBUG ("TX POLL -- rid:{:d}, seq:{:d}", pdu.rid(), p->seqid());
    }

    std::string psdu;
    if (pdu.SerializeToString (&psdu))
    {
      update_airtime (pdu.rid(), psdu.size());
      m_phy->send (std::move(psdu));
    }
    else
      FSM_PROCESS_EVENT(ev_error());
  }
}

void mac::send_u_ack_pdu (weak_ptr<rm_ctx> wrm)
{
  WIFLX_LOG_FUNCTION (this);

  common::messages::mac::appdu pdu;

  uint16_t rid = common::messages::mac::BROADCAST_RID;
  if (auto rm = wrm.lock ())
  {
    rid = rm->m_rid;
  }

  {
    m_poll_rid = 0;

    {
      lock_guard<mutex> lock (m_ack_mutex);
      for (auto it = m_acks.begin(); it != m_acks.end();)
      {
        auto &pair = *it;
        if (pair.first != rid)
        {
          WIFLX_LOG_DEBUG ("TX U ACK -- rid:{:d}, seq:{:d}", pair.first, pair.second.seqid());
          pdu.set_rid (pair.first);
          auto *a = pdu.add_a ();
          a->Swap (&pair.second);
          it = m_acks.erase (it);
        }
        else
          ++it;
      }
    }

    if (pdu.rid())
    {
      std::string psdu;
      if (pdu.SerializeToString (&psdu))
      {
        update_airtime (pdu.rid(), psdu.size());
        FSM_PROCESS_EVENT (ev_send(true));
        m_phy->send (std::move(psdu));
      }
      else
        FSM_PROCESS_EVENT(ev_error());
    }
  }
}

void mac::send_pdu (std::weak_ptr<rm_ctx> wrm, pktsptr plp)
{
  WIFLX_LOG_FUNCTION (this << plp);

  FSM_PROCESS_EVENT(ev_send (wrm, plp));
}

void mac::fsm_sched ()
{
  WIFLX_LOG_FUNCTION (this);

  schedule ();
}

void mac::fsm_error ()
{
  WIFLX_LOG_FUNCTION (this);

  schedule_retry ();
}

void mac::fsm_p_ack_wait ()
{
  WIFLX_LOG_FUNCTION (this << m_cfg.p_ack_timeout);

  m_ack_timer.expires_from_now (
    std::chrono::microseconds (m_cfg.p_ack_timeout));
  m_ack_timer.async_wait (
    [this] (const boost::system::error_code &ec)
    {
      if (!ec)
      {
        FSM_PROCESS_EVENT (ev_timeout());
      }
      else if (ec != common::error::operation_aborted)
      {
        WIFLX_LOG_ERROR ("ec: {}", ec.message ());
      }
    });
}

void mac::fsm_d_ack_wait ()
{
  WIFLX_LOG_FUNCTION (this << m_cfg.d_ack_timeout);

  m_ack_timer.expires_from_now (
    std::chrono::microseconds (m_cfg.d_ack_timeout));
  m_ack_timer.async_wait (
    [this] (const boost::system::error_code &ec)
    {
      if (!ec)
      {
        FSM_PROCESS_EVENT (ev_timeout());
      }
      else if (ec != common::error::operation_aborted)
      {
        WIFLX_LOG_ERROR ("ec: {}", ec.message ());
      }
    });
}

void mac::fsm_ra_wait ()
{
  WIFLX_LOG_FUNCTION (this << m_cfg.ra_ack_timeout);

  m_ack_timer.expires_from_now (
    std::chrono::microseconds (m_cfg.ra_ack_timeout));
  m_ack_timer.async_wait (
    [this] (const boost::system::error_code &ec)
    {
      if (!ec)
      {
        FSM_PROCESS_EVENT (ev_timeout());
      }
      else if (ec != common::error::operation_aborted)
      {
        WIFLX_LOG_ERROR ("ec: {}", ec.message ());
      }
    });
}

void mac::fsm_p_ack_timeout ()
{
  WIFLX_LOG_FUNCTION (this);

  {
    unique_lock<mutex> lock (m_rm_map_mutex);
    const auto it = m_rm_map.find (m_poll_rid);
    if (it == m_rm_map.end ())
    {
      WIFLX_LOG_ERROR ("unknown remote: {:d}", m_poll_rid);
    }
    else
    {
      lock.unlock ();
      poll_failure (it->second);
    }
  }

  schedule ();
}

void mac::fsm_d_ack_timeout ()
{
  WIFLX_LOG_FUNCTION (this);

  {
    unique_lock<mutex> lock (m_rm_map_mutex);
    const auto it = m_rm_map.find (m_poll_rid);
    if (it == m_rm_map.end ())
    {
      WIFLX_LOG_ERROR ("unknown remote: {:d}", m_poll_rid);
    }
    else
    {
      lock.unlock ();
      tx_failure (it->second);
    }
  }

  schedule ();
}

void mac::poll_failure (std::weak_ptr<rm_ctx> wrm)
{
  WIFLX_LOG_FUNCTION (this);

  if (auto rm = wrm.lock())
  {
    unique_lock<mutex> lock (rm->m_mutex);
    ++rm->m_poll_retries;
    if (rm->m_poll_retries > m_cfg.max_poll_retries)
    {
      WIFLX_LOG_ERROR ("poll retries exhausted, rid: {:d}\n", rm->m_rid);
      rm->m_poll_retries = 0;
      rm->m_inactivity_timer.cancel ();
      lock.unlock ();
      remove_from_active_list (wrm);
    }
  }
}

void mac::tx_failure (std::weak_ptr<rm_ctx> wrm)
{
  WIFLX_LOG_FUNCTION (this);

  if (auto rm = wrm.lock())
  {
    unique_lock<mutex> lock (rm->m_mutex);
    ++rm->m_data_retries;
    if (rm->m_data_retries > m_cfg.max_data_retries)
    {
      WIFLX_LOG_ERROR ("data retries exhausted, rid: {:d}\n", rm->m_rid);
      rm->m_data_retries = 0;
      rm->m_inactivity_timer.cancel ();
      lock.unlock ();
      remove_from_active_list (wrm);
      release_all_packets ();
    }
  }
}

void mac::tx_success (weak_ptr<rm_ctx> wrm, const common::messages::mac::ack &a)
{
  WIFLX_LOG_FUNCTION (this);

  if (auto rm = wrm.lock())
  {
    lock_guard<mutex> lock (rm->m_mutex);
    rm->m_data_retries = 0;
  }

  release_packet (a);
}

//
// IWF -> MAC
//
void mac::send (const uint16_t rid, const uint8_t tid, const uint32_t fid, std::string &&msdu)
{
  WIFLX_LOG_FUNCTION (this << rid << unsigned(tid) << fid << msdu.size ());

  WIFLX_PROFILING_SCOPE_N("mac_send");

  unique_lock<mutex> lock (m_rm_map_mutex);
  const auto it = m_rm_map.find (rid);
  if (it == m_rm_map.end ())
  {
    WIFLX_LOG_ERROR ("unknown remote: {:d}", rid);
    return;
  }
  auto &rm = it->second;
  unique_lock<mutex> lock2 (rm->m_mutex);
  lock.unlock ();

  // activate this remote so it can be scheduled
  if (!rm->m_active)
  {
    WIFLX_LOG_DEBUG ("activating remote: {:d}", rid);
    rm->m_active = true;
    rm->m_deficit = m_cfg.rm_airtime_quantum;
    {
      lock_guard<mutex> lock3 (m_rm_list_mutex);
      m_new_rms.push_back (rm);
    }
  }

  packet pkt (std::move(msdu), ++rm->m_seqid, rm);
  lock2.unlock ();

  m_fq_codel.enqueue (tid, fid, rid, std::move(pkt));
}

//
// PHY -> MAC
//
void mac::on_send (const size_t bytes_sent)
{
  WIFLX_LOG_FUNCTION (this << bytes_sent);

  WIFLX_PROFILING_SCOPE_N("mac_on_send");

  FSM_PROCESS_EVENT (ev_sent ());
}

void mac::on_receive (std::string &&mpdu, const stats &st)
{
  WIFLX_LOG_FUNCTION (this << mpdu.size () << st.m_valid);

  WIFLX_PROFILING_SCOPE_N("mac_on_receive");

  const auto now = common::clock::now ();

  if (st.m_valid)
  {
    common::messages::mac::rmpdu pdu;
    if (pdu.ParseFromString (mpdu))
    {
      if (is_bcast_rid(m_poll_rid) || pdu.rid() == m_poll_rid)
      {
        if (m_ack_timer.cancel ())
        {
          update_airtime (pdu.rid(), mpdu.size ());

          if (pdu.has_ra ())
            process_ra (pdu, now);

          if (pdu.rid() == m_poll_rid)
          {
            if (pdu.a_size ())
              process_ack (pdu, now);

            if (pdu.d_size ())
              process_data (pdu, now);
          }

          FSM_PROCESS_EVENT (ev_rcvd());
        }
        else
          WIFLX_LOG_ERROR ("ACK TIMER ALREADY EXPIRED");
      }
      else
        WIFLX_LOG_ERROR ("INVALID TRANSMISSION BY {:d}", pdu.rid());
    }
    else
    {
      WIFLX_LOG_ERROR ("failed to parse mpdu");
    }
  }
  else
    WIFLX_LOG_ERROR ("invalid mpdu");
}

void mac::process_ra (
  const common::messages::mac::rmpdu &pdu,
  const common::clock::time_point &now)
{
  WIFLX_LOG_FUNCTION (this << pdu.rid());

  WIFLX_LOG_DEBUG ("RX RA -- rid:{:d}, seq:{:d}", pdu.rid(), pdu.ra().seqid ());

  if (admission_control_check (pdu.ra()))
  {
    unique_lock<mutex> lock (m_rm_map_mutex);
    auto it = m_rm_map.find (pdu.rid());
    if (it == m_rm_map.end ())
    {
      try
      {
        auto rm = std::make_shared<rm_ctx> (pdu.rid(), m_cfg.rm_airtime_quantum, m_timer_ioc);
        const auto result = m_rm_map.emplace (pdu.rid(), rm);
        if (!result.second)
          throw std::runtime_error ("duplicate rid");
        it = result.first;
      }
      catch (const std::exception &ex)
      {
        lock.unlock ();

        WIFLX_LOG_ERROR ("{}", ex.what ());

        common::messages::mac::ack a;
        a.set_seqid (pdu.ra().seqid ());
        a.set_error (common::messages::mac::ER_INSUFFICIENT_RESOURCES);
        enqueue_ack (pdu.rid (), a);
        return;
      }
    }
    auto &rm = it->second;
    unique_lock<mutex> lock2 (rm->m_mutex);
    lock.unlock ();

    rm->m_bsr = pdu.bsr ();
    WIFLX_LOG_DEBUG ("BSR: {:d}", rm->m_bsr);

    // activate this remote so it can be scheduled
    if (!rm->m_active)
    {
      rm->m_active = true;
      rm->m_deficit = m_cfg.rm_airtime_quantum;
      {
        lock_guard<mutex> lock3 (m_rm_list_mutex);
        m_new_rms.push_back (rm);
      }
    }
    lock2.unlock ();

    if (m_cfg.inactivity_timeout)
      reset_inactivity_timer (rm);

    common::messages::mac::ack a;
    a.set_seqid (pdu.ra().seqid ());
    enqueue_ack (pdu.rid (), a);
  }
  else
  {
    common::messages::mac::ack a;
    a.set_seqid (pdu.ra().seqid ());
    a.set_error (common::messages::mac::ER_INSUFFICIENT_RESOURCES);
    enqueue_ack (pdu.rid(), a);
  }
}

bool mac::admission_control_check (const common::messages::mac::rach &ra)
{
  WIFLX_LOG_FUNCTION (this);

  // Normally one would perform admission control here to ensure
  // constraints on system performance (avg per remote latency, ul/dl
  // throughout etc) are not violated if the new remote is accepted
  // into the system. In this simple scheduler we admit everyone :)
  return true;
}

void mac::process_data (
  const common::messages::mac::rmpdu &pdu,
  const common::clock::time_point &now)
{
  WIFLX_LOG_FUNCTION (this << pdu.rid ());

  unique_lock<mutex> lock (m_rm_map_mutex);
  auto it = m_rm_map.find (pdu.rid());
  if (it == m_rm_map.end ())
  {
    WIFLX_LOG_ERROR ("unknown remote: {:d}", pdu.rid());
    return;
  }
  auto &rm = it->second;
  unique_lock<mutex> lock2 (rm->m_mutex);
  lock.unlock ();

  rm->m_poll_retries = 0;
  rm->m_bsr = pdu.bsr ();
  WIFLX_LOG_DEBUG ("BSR: {:d}", rm->m_bsr);
  lock2.unlock ();

  for (int i = 0; i < pdu.d_size (); ++i)
  {
    auto &d = pdu.d(i);

    WIFLX_LOG_DEBUG ("RX DATA -- rid:{:d}, seq:{:d}, size:{:d}",
      pdu.rid(), d.seqid (), d.payload().size());

    common::messages::mac::ack a;
    a.set_seqid (d.seqid ());
    enqueue_ack (pdu.rid(), a);

    const bool null_frame = d.payload ().empty ();
    if (!null_frame)
    {
      if (m_cfg.inactivity_timeout)
        reset_inactivity_timer (rm);

      m_iwf->on_receive (pdu.rid(), d.payload ());
    }
  }
}

void mac::process_ack (
  const common::messages::mac::rmpdu &pdu,
  const common::clock::time_point &now)
{
  WIFLX_LOG_FUNCTION (this << pdu.rid());

  unique_lock<mutex> lock (m_rm_map_mutex);
  auto it = m_rm_map.find (pdu.rid());
  if (it == m_rm_map.end ())
  {
    WIFLX_LOG_ERROR ("unknown remote: {:d}", pdu.rid());
    return;
  }
  auto &rm = it->second;
  lock.unlock ();

  for (int i = 0; i < pdu.a_size (); ++i)
  {
    auto &a = pdu.a(i);

    WIFLX_LOG_DEBUG ("RX ACK -- rid:{:d}, seq:{:d}", pdu.rid(), a.seqid ());

    if (check_ack (a))
    {
      if (a.error ())
      {
        WIFLX_LOG_ERROR ("ack error: {}", a.error ());
        tx_failure (rm);
      }
      else
      {
        tx_success (rm, a);
      }
    }
  }
}

bool mac::check_ack (const common::messages::mac::ack &a)
{
  bool retval = false;

  lock_guard<mutex> lock (m_txq_mutex);
  for (const auto &pkt : m_retry_txq.m_q)
  {
    retval = (pkt.m_seqid == a.seqid ());
    if (retval)
      break;
  }
  if (!retval)
    WIFLX_LOG_ERROR ("invalid seqid: {:d}", a.seqid());

  return retval;
}

void mac::reset_inactivity_timer (std::shared_ptr<rm_ctx> rm)
{
  WIFLX_LOG_FUNCTION (this);

  {
    std::weak_ptr<rm_ctx> wrm (rm);
    lock_guard<mutex> lock (rm->m_mutex);
    rm->m_inactivity_timer.expires_from_now (
      std::chrono::seconds (m_cfg.inactivity_timeout));
    rm->m_inactivity_timer.async_wait (
      [this, wrm] (const boost::system::error_code &ec)
      {
        if (!ec)
        {
          if (auto rm = wrm.lock ())
          {
            {
              lock_guard<mutex> lock (rm->m_mutex);
              enqueue_release (packet (++rm->m_seqid, rm));
            }
            remove_from_active_list (rm);
            WIFLX_LOG_INFO ("INACTIVITY TIMER EXPIRED -- rid:{:d}", rm->m_rid);
          }
        }
        else if (ec != common::error::operation_aborted)
        {
          WIFLX_LOG_ERROR("ec: %s", ec.message());
        }
      });
  }
}

void mac::remove_from_active_list (std::weak_ptr<rm_ctx> wrm)
{
  WIFLX_LOG_FUNCTION (this);

  {
    lock_guard<mutex> lock (m_rm_list_mutex);

    m_new_rms.remove_if (
      [wrm] (const std::weak_ptr<rm_ctx> &val)
      {
        return val.lock () == wrm.lock ();
      });

    m_old_rms.remove_if (
      [wrm] (const std::weak_ptr<rm_ctx> &val)
      {
        return val.lock () == wrm.lock ();
      });
  }

  if (auto rm = wrm.lock())
  {
    lock_guard<mutex> lock2 (rm->m_mutex);
    rm->m_active = false;
  }
}

void mac::enqueue_retry (packet &&p)
{
  WIFLX_LOG_FUNCTION(this);

  lock_guard<mutex> lock (m_txq_mutex);
  m_retry_txq.push (std::move(p));
}

void mac::enqueue_release (packet &&p)
{
  WIFLX_LOG_FUNCTION(this);

  lock_guard<mutex> lock (m_txq_mutex);
  m_release_txq.push (std::move(p));
}

void mac::enqueue_ack (const uint16_t rid, const common::messages::mac::ack &a)
{
  WIFLX_LOG_FUNCTION(this);

  lock_guard<mutex> lock (m_ack_mutex);
  m_acks.emplace_back (rid, a);
}

void mac::release_packet (const common::messages::mac::ack &a)
{
  WIFLX_LOG_FUNCTION (this);

  lock_guard<mutex> lock (m_txq_mutex);
  m_retry_txq.m_q.remove_if (
    [&a] (const wiflx::common::packet &pkt)
    {
      return a.seqid() == pkt.m_seqid;
    });
}

void mac::release_all_packets ()
{
  WIFLX_LOG_FUNCTION (this);

  lock_guard<mutex> lock (m_txq_mutex);
  m_retry_txq.clear ();
}

void mac::update_airtime (const uint16_t rid, const size_t size)
{
  WIFLX_LOG_FUNCTION (this << rid);

  unique_lock<mutex> lock (m_rm_map_mutex);
  auto it = m_rm_map.find (rid);
  if (it == m_rm_map.end ())
  {
    WIFLX_LOG_ERROR ("unknown remote: {:d}", rid);
    return;
  }
  auto &rm = it->second;
  lock_guard<mutex> lock2 (rm->m_mutex);
  lock.unlock ();
  rm->m_deficit -= size;
  m_b_poll_airtime_deficit -= size;
}

void mac::timer_run ()
{
  WIFLX_LOG_FUNCTION (this);

  WIFLX_PROFILING_SETTHREADNAME("MAC TIMER THREAD");

  m_timer_ioc.run ();
}

} // namespace ap
} // namespace wiflx
