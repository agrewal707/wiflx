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

#include <rm/mac.h>

#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_VECTOR_SIZE 30
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>
#include <boost/msm/front/euml/state_grammar.hpp>
#include <common/log.h>
#include <common/error.h>
#include <common/utils.h>

namespace wiflx {
namespace rm {
namespace msm = boost::msm;
namespace mpl = boost::mpl;
using namespace msm::front;
// for And_ operator
using namespace msm::front::euml;
using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::mutex;
using std::unique_lock;
using std::lock_guard;

//
// MAC FSM
//
struct ev_poll
{
  explicit ev_poll (const uint16_t rid, const common::messages::mac::poll &p) :
    m_rid (rid),
    m_p (p)
  {}

  uint16_t m_rid;
  common::messages::mac::poll m_p;
};
struct ev_sent {};
struct ev_timeout {};
struct ev_nak
{
  ev_nak (const common::messages::mac::ack &a):
    m_ack (a)
  {}

  common::messages::mac::ack m_ack;
};
struct ev_ack
{
  ev_ack (const common::messages::mac::ack &a):
    m_ack (a)
  {}

  common::messages::mac::ack m_ack;
};
struct ev_release
{
  ev_release ():
    m_from_ap (false)
  {}

  ev_release (bool from_ap):
    m_from_ap (from_ap)
  {}

  bool m_from_ap;
};

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
  DECLARE_ST (st_ra_backoff);
  DECLARE_ST (st_sending_ra);
  DECLARE_ST (st_ra_sent);
  DECLARE_ST (st_active);
  DECLARE_ST (st_sending_data);

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
  struct is_ra_poll
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    bool operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_TRACE_L1("GD: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      return fsm.m_mac->fsm_is_ra_poll (ev.m_rid);
    }
  };

  struct is_data_poll
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    bool operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_TRACE_L1("GD: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      return fsm.m_mac->fsm_is_data_poll (ev.m_rid);
    }
  };

  struct is_data_pending
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    bool operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_TRACE_L1("GD: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      return fsm.m_mac->fsm_is_data_pending ();
    }
  };

  struct is_ra_backoff_done
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    bool operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_TRACE_L1("GD: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      return fsm.m_mac->fsm_is_ra_backoff_done();
    }
  };

  struct is_ra_ack
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    bool operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_TRACE_L1("GD: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      return fsm.m_mac->fsm_is_ra_ack (ev.m_ack);
    }
  };

  struct is_data_ack
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    bool operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_TRACE_L1("GD: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      return fsm.m_mac->fsm_is_data_ack (ev.m_ack);
    }
  };

  //
  // Transitions
  //
  struct tr_proc_ra_poll
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src,TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_proc_ra_poll (ev.m_p);
    }
  };

  struct tr_init_ra_backoff
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src,TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_init_ra_backoff (ev.m_p);
    }
  };

  struct tr_proc_ra_backoff
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src,TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_proc_ra_backoff (ev.m_p);
    }
  };

  struct tr_send_ra
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src,TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_send_ra ();
    }
  };

  struct tr_send_data
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      return fsm.m_mac->fsm_send_data ();
    }
  };

  struct tr_send_ack
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      return fsm.m_mac->fsm_send_ack ();
    }
  };

  struct tr_wait
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      return fsm.m_mac->fsm_wait ();
    }
  };
  struct tr_proc_ra_timeout
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_WARNING("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());
    }
  };
  struct tr_proc_data_timeout
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_WARNING("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      return fsm.m_mac->fsm_proc_data_timeout ();
    }
  };

  struct tr_proc_data_nak
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_WARNING("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_proc_data_nak (ev.m_ack);
    }
  };

  struct tr_proc_data_ack
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      fsm.m_mac->fsm_proc_data_ack (ev.m_ack);
    }
  };

  struct tr_release
  {
    template <class Fsm, class Evt, class SourceState, class TargetState>
    void operator() (Evt const &ev, Fsm &fsm, SourceState &src, TargetState &dst)
    {
      WIFLX_LOG_INFO("TR: ev: {}, src: {}, dst: {}", typeid(ev).name(), src.name(), dst.name());

      return fsm.m_mac->fsm_release (ev.m_from_ap);
    }
  };

  // Transition table for MAC operation
  struct transition_table : mpl::vector<
    //    Start             Event                 Next              Action                Guard
    //  +------------------+-------------------- +-----------------+---------------------+-------+
    Row < st_idle,          ev_poll,          st_idle,          tr_proc_ra_poll,      And_<is_ra_poll,Not_<is_data_pending>> >,
    Row < st_idle,          ev_poll,          st_ra_backoff,    tr_init_ra_backoff,   And_<is_ra_poll,is_data_pending> >,
    Row < st_idle,          ev_poll,          st_sending_data,  tr_send_data,         is_data_poll>,
    Row < st_idle,          ev_sent,          none,             none,                 none>,

    Row < st_ra_backoff,    ev_poll,          st_ra_backoff,    tr_proc_ra_backoff,   And_<is_ra_poll,And_<is_data_pending,Not_<is_ra_backoff_done>>> >,
    Row < st_ra_backoff,    ev_poll,          st_sending_ra,    tr_send_ra,           And_<is_ra_poll,And_<is_data_pending,is_ra_backoff_done>> >,
    Row < st_ra_backoff,    ev_poll,          st_sending_ra,    tr_send_ra,           is_data_poll >,
    Row < st_ra_backoff,    ev_release,       st_idle,          none,                 none >,

    Row < st_sending_ra,    ev_sent,          st_ra_sent,       tr_wait,              none >,

    Row < st_ra_sent,       ev_poll,          st_sending_ra,    tr_send_ra,           is_data_poll >,
    Row < st_ra_sent,       ev_timeout,       st_idle,          tr_proc_ra_timeout,   none >,
    Row < st_ra_sent,       ev_nak,           st_idle,          none,                 is_ra_ack >,
    Row < st_ra_sent,       ev_ack,           st_active,        none,                 is_ra_ack >,
    Row < st_ra_sent,       ev_release,       st_idle,          tr_release,           none >,

    Row < st_sending_data,  ev_sent,          st_active,        tr_wait,              none >,

    Row < st_active,        ev_poll,          st_sending_data,  tr_send_data,         is_data_poll >,
    Row < st_active,        ev_timeout,       st_active,        tr_proc_data_timeout, none >,
    Row < st_active,        ev_nak,           st_active,        tr_proc_data_nak,     is_data_ack >,
    Row < st_active,        ev_ack,           st_active,        tr_proc_data_ack,     is_data_ack >,
    Row < st_active,        ev_release,       st_idle,          tr_release,           none >
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

// helpers
#define M_FSM (std::static_pointer_cast<operation>(m_fsm))
#define FSM_PROCESS_EVENT(x) M_FSM->process_event(x);

//
// MAC
//
mac::mac (
  const config::mac &cfg,
  const wiflx::common::config::fq_codel &fq_codel_cfg,
  const wiflx::common::config::codel &codel_cfg):
  m_cfg (cfg),
  m_phy (nullptr),
  m_iwf (nullptr),
  m_fsm (std::make_shared<operation> (this)),
  m_fq_codel (fq_codel_cfg, codel_cfg),
  m_rid (cfg.rid),
  m_data_retries (0),
  m_ra_backoff_counter (0),
  m_last_poll_seqid (0),
  m_ra_seqid (0),
  m_data_seqid (0),
  m_timer_ioc (),
  m_timer_ioc_work (boost::asio::make_work_guard(m_timer_ioc)),
  m_ack_timer (m_timer_ioc),
  m_rlf_timer (m_timer_ioc),
  m_stopped (true)
{
  WIFLX_LOG_FUNCTION (this);

  // TODO- tid -> cw window min/max
  m_ra_cw_map[0] = ra_cw (m_cfg.cw_min, m_cfg.cw_max);
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
    M_FSM->start();
    m_stopped = false;
  }
}

void mac::stop ()
{
  WIFLX_LOG_FUNCTION (this);

  if (!m_stopped)
  {
    m_stopped = true;
    m_ack_timer.cancel ();
    m_rlf_timer.cancel ();
    m_timer_ioc_work.reset();
  }
}

void mac::enqueue_retry (packet &&p)
{
  WIFLX_LOG_FUNCTION(this);

  lock_guard<mutex> lock (m_txq_mutex);
  m_retry_txq.push (std::move(p));
}

void mac::enqueue_ack (const common::messages::mac::ack &a)
{
  WIFLX_LOG_FUNCTION(this);

  lock_guard<mutex> lock (m_ack_mutex);
  m_acks.push_back (a);
}

wiflx::common::packets mac::dequeue ()
{
  WIFLX_LOG_FUNCTION (this);

  wiflx::common::packets pkts;
  {
    lock_guard<mutex> lock (m_txq_mutex);
    // return only one packet when retrying
    if (!m_retry_txq.empty ())
    {
      auto pkt = std::move (m_retry_txq.front ());
      m_retry_txq.pop ();
      pkts.push_back(std::move (pkt));
      return pkts;
    }
  }

  {
    pkts = m_fq_codel.dequeue (0);
    if (!pkts.size ())
    {
      // return null packet when queue is empty
      pkts.push_back (std::move(packet()));
    }
  }

  return pkts;
}

void mac::release_packet (const common::messages::mac::ack &a)
{
  WIFLX_LOG_FUNCTION (this);

  lock_guard<mutex> lock (m_txq_mutex);

  auto it = std::find_if (m_retry_txq.m_q.begin (), m_retry_txq.m_q.end (),
    [&a] (const packet &pkt)
    {
      return a.seqid() == pkt.m_seqid;
    });

  if (it != m_retry_txq.m_q.end())
  {
    m_retry_txq.m_bytes -= it->size ();
    m_retry_txq.m_q.erase (it);
  }
}

void mac::release_all_packets ()
{
  WIFLX_LOG_FUNCTION (this);

  lock_guard<mutex> lock (m_txq_mutex);
  m_retry_txq.clear ();
}

//
// IWF -> MAC
//
void mac::send (const uint8_t tid, const uint32_t fid, std::string &&msdu)
{
  WIFLX_LOG_FUNCTION (this << unsigned(tid) << fid << msdu.size ());

  m_fq_codel.enqueue (tid, fid, 0, packet (std::move(msdu),++m_data_seqid));
}

//
// PHY -> MAC
//
void mac::on_send ()
{
  WIFLX_LOG_FUNCTION (this);

  FSM_PROCESS_EVENT(ev_sent());
}

void mac::on_receive (std::string &&mpdu, const stats &st)
{
  WIFLX_LOG_FUNCTION (this << mpdu.size () << st.m_valid);

  const auto now = common::clock::now ();

  if (st.m_valid)
  {
    common::messages::mac::appdu pdu;
    if (pdu.ParseFromString (mpdu))
    {
      // Process MAC messages (in this order)
      if (pdu.a_size ())
        process_ack (pdu, now);

      if (pdu.d_size ())
        process_data (pdu, now);

      if (pdu.has_p ())
        process_poll (pdu, now);
    }
    else
      WIFLX_LOG_ERROR ("failed to parse mpdu");
  }
  else
    WIFLX_LOG_ERROR ("invalid mpdu");
}

void mac::process_ack (
  const common::messages::mac::appdu &pdu,
  const common::clock::time_point &now)
{
  WIFLX_LOG_FUNCTION (this << pdu.rid());

  if (pdu.rid() == m_rid)
  {
    if (m_ack_timer.cancel ())
    {
      for (int i = 0; i < pdu.a_size (); ++i)
      {
        auto &a = pdu.a(i);

        WIFLX_LOG_DEBUG ("RX ACK -- rid:{:d}, seq:{:d}", pdu.rid(), a.seqid ());

        if (a.error ())
        {
          WIFLX_LOG_ERROR ("ack error: {}", a.error ());
          FSM_PROCESS_EVENT (ev_nak (a));
        }
        else
        {
          FSM_PROCESS_EVENT (ev_ack (a));
        }
      }
    }
    else
      WIFLX_LOG_ERROR ("ACK TIMER ALREADY EXPIRED");
  }
}

void mac::process_data (
  const common::messages::mac::appdu &pdu,
  const common::clock::time_point &now)
{
  WIFLX_LOG_FUNCTION (this << pdu.rid());

  if (pdu.rid() == m_rid)
  {
    for (int i = 0; i < pdu.d_size (); ++i)
    {
      auto &d = pdu.d(i);
      bool null_frame = d.payload ().empty ();
      if (null_frame)
      {
        WIFLX_LOG_DEBUG ("RX RELEASE -- rid:{:d}, seq:{:d}, size:{:d}",
          pdu.rid(), d.seqid(), d.payload().size());

        // set pending data ack
        common::messages::mac::ack a;
        a.set_seqid (d.seqid ());
        enqueue_ack (a);

        FSM_PROCESS_EVENT (ev_release(true));
      }
      else
      {
        WIFLX_LOG_DEBUG ("RX DATA -- rid:{:d}, seq:{:d}, size:{:d}",
          pdu.rid (), d.seqid (), d.payload().size());

        // set pending data ack
        common::messages::mac::ack a;
        a.set_seqid (d.seqid ());
        enqueue_ack (a);

        m_iwf->on_receive (d.payload ());
      }
    }
  }
  else if (pdu.rid () == common::messages::mac::BROADCAST_RID)
  {
    for (int i = 0; i < pdu.d_size (); ++i)
    {
      auto &d = pdu.d(i);

      WIFLX_LOG_DEBUG ("RX DATA -- rid:{:d}, seq:{:d}, size:{:d}",
        pdu.rid (), d.seqid (), d.payload().size());

      m_iwf->on_receive (d.payload ());
    }
  }
  // else not for us
}

void mac::process_poll (
  const common::messages::mac::appdu &pdu,
  const common::clock::time_point &now)
{
  WIFLX_LOG_FUNCTION (this << pdu.rid ());

  WIFLX_LOG_DEBUG ("RX POLL -- rid:{:d}, seq:{:d}", pdu.rid (), pdu.p().seqid ());

  FSM_PROCESS_EVENT(ev_poll(pdu.rid(), pdu.p()));
}

//
// FSM
//
bool mac::fsm_is_ra_poll (const uint16_t rid)
{
  WIFLX_LOG_FUNCTION(this);

  return rid == common::messages::mac::BROADCAST_RID;
}

bool mac::fsm_is_data_poll (const uint16_t rid)
{
  WIFLX_LOG_FUNCTION (this);

  return rid == m_rid;
}

bool mac::fsm_is_data_pending ()
{
  WIFLX_LOG_FUNCTION(this << m_fq_codel.get_total_buffered_bytes());

  return m_fq_codel.get_total_buffered_bytes() > 0;
}

bool mac::fsm_is_ra_backoff_done ()
{
  WIFLX_LOG_FUNCTION (this);

  return m_ra_backoff_counter <= 0;
}

bool mac::fsm_is_ra_ack (const common::messages::mac::ack &a)
{
  WIFLX_LOG_FUNCTION (this);

  const auto retval = (m_ra_seqid == a.seqid ());
  if (!retval)
    WIFLX_LOG_ERROR ("invalid seqid: {:d}", a.seqid());

  return retval;
}

bool mac::fsm_is_data_ack (const common::messages::mac::ack &a)
{
  WIFLX_LOG_FUNCTION (this);

  lock_guard<mutex> lock (m_txq_mutex);
  auto it = std::find_if (m_retry_txq.m_q.begin (), m_retry_txq.m_q.end (),
    [&a] (const packet &pkt)
    {
      return a.seqid() == pkt.m_seqid;
    });

  if (it != m_retry_txq.m_q.end())
  {
    return true;
  }

  WIFLX_LOG_ERROR ("invalid seqid: {:d}", a.seqid());
  return false;;
}

void mac::fsm_proc_ra_poll (const common::messages::mac::poll &p)
{
  WIFLX_LOG_FUNCTION (this);

  m_last_poll_seqid = p.seqid ();
}

void mac::fsm_init_ra_backoff (const common::messages::mac::poll &p)
{
  WIFLX_LOG_FUNCTION (this);

  m_last_poll_seqid = p.seqid ();

  uint8_t tid = 0;
  // TODO
  // auto tid = m_fq_codel.get_next_tid ();

  const auto it = m_ra_cw_map.find (tid);
  if (it != m_ra_cw_map.end())
  {
    // subtract one to account for first POLL
    m_ra_backoff_counter = common::get_random_number(it->second.m_min, it->second.m_max) - 1;
    WIFLX_LOG_INFO ("backoff counter {:d}", m_ra_backoff_counter);

    if (fsm_is_ra_backoff_done())
    {
      FSM_PROCESS_EVENT(ev_poll(m_rid, p));
    }
  }
  else
  {
    WIFLX_LOG_ERROR ("unknown tid: {:d}", tid);
  }
}

void mac::fsm_proc_ra_backoff (const common::messages::mac::poll &p)
{
  WIFLX_LOG_FUNCTION (this);

  m_ra_backoff_counter -= p.seqid () - m_last_poll_seqid;
  WIFLX_LOG_INFO ("backoff counter {:d}", m_ra_backoff_counter);
}

void mac::fsm_send_ra ()
{
  WIFLX_LOG_FUNCTION (this);

  common::messages::mac::rmpdu pdu;
  pdu.set_rid (m_rid);

  // RACH
  {
    auto *ra = pdu.mutable_ra ();
    ra->set_seqid (++m_ra_seqid);
    WIFLX_LOG_DEBUG("TX RA -- seq:{:d}", ra->seqid());
  }

  // pending ACKs
  {
    lock_guard<mutex> lock (m_ack_mutex);
    while (!m_acks.empty())
    {
      auto &ack = m_acks.front ();
      auto *a = pdu.add_a ();
      a->Swap (&ack);
      m_acks.pop_front ();
      WIFLX_LOG_DEBUG ("TX ACK -- seq:{:d}", a->seqid());
    }
  }

  pdu.set_bsr ((m_fq_codel.get_total_buffered_bytes () + 255)/256);

  std::string psdu;
  if (pdu.SerializeToString (&psdu))
    m_phy->send (std::move(psdu));
  else
    WIFLX_LOG_ERROR ("failed to serialize pdu");
}

void mac::fsm_send_data ()
{
  WIFLX_LOG_FUNCTION (this);

  common::messages::mac::rmpdu pdu;
  pdu.set_rid (m_rid);

  // DATA
  auto pkts = dequeue ();
  for (auto &pkt : pkts)
  {
    auto *d = pdu.add_d ();
    d->set_seqid (pkt.m_seqid);
    auto *payload = d->mutable_payload ();
    // don't move out payload, since we need to add this to retry txq.
    *payload = pkt.m_msdu;

    // add this packet to retry queue
    enqueue_retry (std::move (pkt));

    WIFLX_LOG_DEBUG ("TX DATA -- seq:{:d}, size:{:d}", d->seqid(), payload->size ());
  }

  // pending ACKs
  {
    lock_guard<mutex> lock (m_ack_mutex);
    while (!m_acks.empty())
    {
      auto &ack = m_acks.front ();
      auto *a = pdu.add_a ();
      a->Swap (&ack);
      m_acks.pop_front ();
      WIFLX_LOG_DEBUG ("TX ACK -- seq:{:d}", a->seqid());
    }
  }

  pdu.set_bsr ((m_fq_codel.get_total_buffered_bytes () + 255)/256);

  std::string psdu;
  if (pdu.SerializeToString (&psdu))
    m_phy->send (std::move(psdu));
  else
    WIFLX_LOG_ERROR ("failed to serialize pdu");

  reset_rlf_timer ();
}

void mac::fsm_send_ack ()
{
  WIFLX_LOG_FUNCTION (this);

  common::messages::mac::rmpdu pdu;
  pdu.set_rid (m_rid);
  {
    lock_guard<mutex> lock (m_ack_mutex);
    while (!m_acks.empty())
    {
      auto &ack = m_acks.front ();
      auto *a = pdu.add_a ();
      a->Swap (&ack);
      m_acks.pop_front ();
      WIFLX_LOG_DEBUG ("TX ACK -- seq:{:d}", a->seqid());
    }
  }

  if (pdu.a_size ())
  {
    std::string psdu;
    if (pdu.SerializeToString (&psdu))
      m_phy->send (std::move(psdu));
    else
      WIFLX_LOG_ERROR ("failed to serialize pdu");
  }
}

void mac::fsm_wait ()
{
  WIFLX_LOG_FUNCTION (this);

  m_ack_timer.expires_from_now (
    std::chrono::microseconds (m_cfg.ack_timeout));
  m_ack_timer.async_wait (
    [this] (const boost::system::error_code &ec)
    {
      if (!ec)
      {
        FSM_PROCESS_EVENT(ev_timeout());
      }
      else if (ec != common::error::operation_aborted)
      {
        WIFLX_LOG_ERROR ("ec: {}", ec.message());
        FSM_PROCESS_EVENT(ev_release());
      }
    });
}

void mac::fsm_proc_data_timeout ()
{
  WIFLX_LOG_FUNCTION (this);

  ++m_data_retries;
  if (m_data_retries > m_cfg.max_data_retries)
  {
    WIFLX_LOG_ERROR ("data retries exhausted");
    release_all_packets ();
    FSM_PROCESS_EVENT (ev_release ());
  }
}

void mac::fsm_proc_data_nak (const common::messages::mac::ack &a)
{
  WIFLX_LOG_FUNCTION (this);

  ++m_data_retries;
  if (m_data_retries > m_cfg.max_data_retries)
  {
    WIFLX_LOG_ERROR ("data retries exhausted");
    release_all_packets ();
    FSM_PROCESS_EVENT (ev_release ());
  }
}

void mac::fsm_proc_data_ack (const common::messages::mac::ack &a)
{
  WIFLX_LOG_FUNCTION (this);

  m_data_retries = 0;
  release_packet (a);

}

void mac::fsm_release (bool from_ap)
{
  WIFLX_LOG_FUNCTION (this << from_ap);

  m_data_retries = 0;
  m_rlf_timer.cancel ();

  if (from_ap)
  {
    fsm_send_ack ();
  }
}

void mac::reset_rlf_timer ()
{
  WIFLX_LOG_FUNCTION (this);

  m_rlf_timer.expires_from_now (
      std::chrono::seconds (m_cfg.rlf_timeout));
  m_rlf_timer.async_wait (
    [this] (const boost::system::error_code &ec)
    {
      if (!ec)
      {
        WIFLX_LOG_DEBUG ("RLF TIMER EXPIRED");
        FSM_PROCESS_EVENT(ev_release());
      }
      else if (ec != common::error::operation_aborted)
      {
        WIFLX_LOG_ERROR("ec: {}", ec.message());
        FSM_PROCESS_EVENT(ev_release());
      }
    });
}


void mac::timer_run ()
{
  WIFLX_LOG_FUNCTION (this);

  m_timer_ioc.run ();
}

} // namespace rm
} // namespace wiflx
