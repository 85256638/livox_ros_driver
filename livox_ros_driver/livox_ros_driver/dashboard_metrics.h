#ifndef LIVOX_ROS_DRIVER_DASHBOARD_METRICS_H_
#define LIVOX_ROS_DRIVER_DASHBOARD_METRICS_H_

#include <stdint.h>

#include <deque>
#include <string>

namespace livox_ros {

/**
 * Cumulative counters sampled by the dashboard.  A counter may reset
 * independently (for example when an SDK session is recreated); Update()
 * treats the new value as the post-reset delta instead of subtracting through
 * zero.  The first sample for a broadcast code is only a baseline.
 */
struct DashboardCounters {
  uint64_t received_packets;
  uint64_t lost_packets;
  uint64_t queue_drops;

  uint64_t handshake_ack_attempts;
  uint64_t handshake_timeout_attempts;
  uint64_t handshake_rejected_attempts;
  uint64_t handshake_network_attempts;
  uint64_t handshake_protocol_attempts;

  uint64_t disconnect_episodes;
  uint64_t handshake_stuck_episodes;
  /** Unique broadcast-only episodes which actually reached the hard-power
   *  state.  This is an incident count, not a request/retry count. */
  uint64_t power_reached_episodes;
  /** Committed entries into the hard-power escalation state.  More than one
   *  entry can occur in one incident, and a late cancellation can prevent an
   *  entry from becoming an outbound request, so this is not an incident
   *  count. */
  uint64_t power_request_edges;
  uint64_t fault_episodes;
  uint64_t reboot_actions;
  uint64_t mode_fail_episodes;

  DashboardCounters()
      : received_packets(0),
        lost_packets(0),
        queue_drops(0),
        handshake_ack_attempts(0),
        handshake_timeout_attempts(0),
        handshake_rejected_attempts(0),
        handshake_network_attempts(0),
        handshake_protocol_attempts(0),
        disconnect_episodes(0),
        handshake_stuck_episodes(0),
        power_reached_episodes(0),
        power_request_edges(0),
        fault_episodes(0),
        reboot_actions(0),
        mode_fail_episodes(0) {}
};

/** A point-in-time view of the two rolling windows. */
struct DashboardWindow {
  std::string broadcast_code;

  /** Valid observation in the retained ten-minute window, capped at 10 min. */
  uint64_t observation_ns;

  /** Traffic and SDK handshake attempts in (now - 60 s, now]. */
  uint64_t received_60s;
  uint64_t lost_60s;
  uint64_t queue_drops_60s;
  uint64_t handshake_ack_60s;
  uint64_t handshake_timeout_60s;
  uint64_t handshake_rejected_60s;
  uint64_t handshake_network_60s;
  uint64_t handshake_protocol_60s;

  /** Episodes/actions in (now - 10 min, now]. */
  uint64_t disconnect_10m;
  uint64_t handshake_stuck_10m;
  uint64_t power_reached_10m;
  uint64_t power_request_edges_10m;
  uint64_t fault_10m;
  uint64_t reboot_10m;
  uint64_t mode_fail_10m;

  /** loss% = lost / (received + lost).  False prevents displaying 0% when
   *  there were no packets with which to calculate a loss rate. */
  bool loss_60s_has_data;
  double loss_60s_percent;

  DashboardWindow()
      : observation_ns(0),
        received_60s(0),
        lost_60s(0),
        queue_drops_60s(0),
        handshake_ack_60s(0),
        handshake_timeout_60s(0),
        handshake_rejected_60s(0),
        handshake_network_60s(0),
        handshake_protocol_60s(0),
        disconnect_10m(0),
        handshake_stuck_10m(0),
        power_reached_10m(0),
        power_request_edges_10m(0),
        fault_10m(0),
        reboot_10m(0),
        mode_fail_10m(0),
        loss_60s_has_data(false),
        loss_60s_percent(0.0) {}

  uint64_t HandshakeErrors60s() const {
    return handshake_timeout_60s + handshake_rejected_60s +
           handshake_network_60s + handshake_protocol_60s;
  }

  bool HasRecentEpisodeOrAction() const {
    return disconnect_10m != 0 || handshake_stuck_10m != 0 ||
           power_reached_10m != 0 || power_request_edges_10m != 0 ||
           fault_10m != 0 || reboot_10m != 0 || mode_fail_10m != 0;
  }
};

/**
 * Per-handle rolling accumulator, isolated by broadcast code and aware of
 * connection/session generations.
 *
 * A handle can be reused by a different lidar.  Passing a different code to
 * Update() clears both windows and establishes a fresh counter baseline, so
 * the old lidar's history can never be attributed to the new one.  A new
 * connection generation for the same code preserves history but starts a new
 * per-connection traffic baseline.  All time arguments must come from the
 * same steady/monotonic nanosecond clock.
 */
class DashboardMetrics {
 public:
  static const int64_t kShortWindowNs = INT64_C(60) * INT64_C(1000000000);
  static const int64_t kLongWindowNs = INT64_C(600) * INT64_C(1000000000);

  DashboardMetrics()
      : initialized_(false),
        connection_generation_(0),
        first_seen_ns_(0),
        last_update_ns_(0) {}

  DashboardWindow Update(const char *broadcast_code,
                         uint64_t connection_generation, int64_t now_ns,
                         const DashboardCounters &counters) {
    return Update(std::string(broadcast_code ? broadcast_code : ""),
                  connection_generation, now_ns, counters);
  }

  DashboardWindow Update(const std::string &broadcast_code,
                         uint64_t connection_generation, int64_t now_ns,
                         const DashboardCounters &counters) {
    /** A code change means this handle now represents another physical lidar.
     *  A backwards steady timestamp indicates a new clock epoch and is also a
     *  safe boundary at which to discard the old rolling window. */
    if (!initialized_ || broadcast_code != broadcast_code_ ||
        now_ns < last_update_ns_) {
      Begin(broadcast_code, connection_generation, now_ns, counters);
      return BuildWindow(now_ns);
    }

    Sample delta;
    delta.when_ns = now_ns;
    if (connection_generation != connection_generation_) {
      /** LidarDevice traffic counters are per connection and ResetLidar()
       *  clears them.  The reset and reconnect can both occur between two 1 Hz
       *  samples; the new counter may already exceed the old counter, in which
       *  case SafeDelta would silently lose all traffic before the old value.
       *  A generation edge makes the entire current traffic value the new
       *  connection's delta.  Link/handshake/incident/action counters live for
       *  the Driver process, so they still use SafeDelta across this edge. */
      delta.counters = DifferenceAcrossGeneration(counters, previous_);
      connection_generation_ = connection_generation;
    } else {
      delta.counters = Difference(counters, previous_);
    }
    previous_ = counters;
    last_update_ns_ = now_ns;

    if (HasAnyDelta(delta.counters)) {
      samples_.push_back(delta);
    }
    Prune(now_ns);
    return BuildWindow(now_ns);
  }

  /** Advance/prune the windows without changing the cumulative baseline. */
  DashboardWindow Read(int64_t now_ns) {
    if (!initialized_) {
      return DashboardWindow();
    }
    /** Treat a backwards clock exactly like Update(): retained timestamps no
     *  longer belong to this steady-clock epoch.  Keep the bound code and
     *  cumulative baseline, but restart observation at the new time. */
    if (now_ns < last_update_ns_) {
      samples_.clear();
      first_seen_ns_ = now_ns;
      last_update_ns_ = now_ns;
    }
    Prune(now_ns);
    return BuildWindow(now_ns);
  }

  void Reset() {
    initialized_ = false;
    broadcast_code_.clear();
    connection_generation_ = 0;
    first_seen_ns_ = 0;
    last_update_ns_ = 0;
    previous_ = DashboardCounters();
    samples_.clear();
  }

 private:
  struct Sample {
    int64_t when_ns;
    DashboardCounters counters;

    Sample() : when_ns(0) {}
  };

  static uint64_t SafeDelta(uint64_t current, uint64_t previous) {
    return current >= previous ? current - previous : current;
  }

  static DashboardCounters Difference(const DashboardCounters &current,
                                      const DashboardCounters &previous) {
    DashboardCounters d;
    d.received_packets =
        SafeDelta(current.received_packets, previous.received_packets);
    d.lost_packets = SafeDelta(current.lost_packets, previous.lost_packets);
    d.queue_drops = SafeDelta(current.queue_drops, previous.queue_drops);
    d.handshake_ack_attempts = SafeDelta(current.handshake_ack_attempts,
                                         previous.handshake_ack_attempts);
    d.handshake_timeout_attempts =
        SafeDelta(current.handshake_timeout_attempts,
                  previous.handshake_timeout_attempts);
    d.handshake_rejected_attempts =
        SafeDelta(current.handshake_rejected_attempts,
                  previous.handshake_rejected_attempts);
    d.handshake_network_attempts =
        SafeDelta(current.handshake_network_attempts,
                  previous.handshake_network_attempts);
    d.handshake_protocol_attempts =
        SafeDelta(current.handshake_protocol_attempts,
                  previous.handshake_protocol_attempts);
    d.disconnect_episodes = SafeDelta(current.disconnect_episodes,
                                      previous.disconnect_episodes);
    d.handshake_stuck_episodes =
        SafeDelta(current.handshake_stuck_episodes,
                  previous.handshake_stuck_episodes);
    d.power_reached_episodes =
        SafeDelta(current.power_reached_episodes,
                  previous.power_reached_episodes);
    d.power_request_edges =
        SafeDelta(current.power_request_edges, previous.power_request_edges);
    d.fault_episodes =
        SafeDelta(current.fault_episodes, previous.fault_episodes);
    d.reboot_actions =
        SafeDelta(current.reboot_actions, previous.reboot_actions);
    d.mode_fail_episodes =
        SafeDelta(current.mode_fail_episodes, previous.mode_fail_episodes);
    return d;
  }

  static DashboardCounters DifferenceAcrossGeneration(
      const DashboardCounters &current, const DashboardCounters &previous) {
    DashboardCounters d = Difference(current, previous);
    d.received_packets = current.received_packets;
    d.lost_packets = current.lost_packets;
    d.queue_drops = current.queue_drops;
    return d;
  }

  static bool HasAnyDelta(const DashboardCounters &d) {
    return d.received_packets != 0 || d.lost_packets != 0 ||
           d.queue_drops != 0 || d.handshake_ack_attempts != 0 ||
           d.handshake_timeout_attempts != 0 ||
           d.handshake_rejected_attempts != 0 ||
           d.handshake_network_attempts != 0 ||
           d.handshake_protocol_attempts != 0 ||
           d.disconnect_episodes != 0 || d.handshake_stuck_episodes != 0 ||
           d.power_reached_episodes != 0 || d.power_request_edges != 0 ||
           d.fault_episodes != 0 || d.reboot_actions != 0 ||
           d.mode_fail_episodes != 0;
  }

  static void AddShort(const DashboardCounters &d, DashboardWindow *out) {
    out->received_60s += d.received_packets;
    out->lost_60s += d.lost_packets;
    out->queue_drops_60s += d.queue_drops;
    out->handshake_ack_60s += d.handshake_ack_attempts;
    out->handshake_timeout_60s += d.handshake_timeout_attempts;
    out->handshake_rejected_60s += d.handshake_rejected_attempts;
    out->handshake_network_60s += d.handshake_network_attempts;
    out->handshake_protocol_60s += d.handshake_protocol_attempts;
  }

  static void AddLong(const DashboardCounters &d, DashboardWindow *out) {
    out->disconnect_10m += d.disconnect_episodes;
    out->handshake_stuck_10m += d.handshake_stuck_episodes;
    out->power_reached_10m += d.power_reached_episodes;
    out->power_request_edges_10m += d.power_request_edges;
    out->fault_10m += d.fault_episodes;
    out->reboot_10m += d.reboot_actions;
    out->mode_fail_10m += d.mode_fail_episodes;
  }

  void Begin(const std::string &broadcast_code,
             uint64_t connection_generation, int64_t now_ns,
             const DashboardCounters &counters) {
    initialized_ = true;
    broadcast_code_ = broadcast_code;
    connection_generation_ = connection_generation;
    first_seen_ns_ = now_ns;
    last_update_ns_ = now_ns;
    previous_ = counters;
    samples_.clear();
  }

  void Prune(int64_t now_ns) {
    if (now_ns < kLongWindowNs) {
      return;
    }
    const int64_t cutoff = now_ns - kLongWindowNs;
    /** Windows are half-open: an event exactly ten minutes old is expired. */
    while (!samples_.empty() && samples_.front().when_ns <= cutoff) {
      samples_.pop_front();
    }
  }

  DashboardWindow BuildWindow(int64_t now_ns) const {
    DashboardWindow out;
    if (!initialized_) {
      return out;
    }

    out.broadcast_code = broadcast_code_;
    if (now_ns >= first_seen_ns_) {
      uint64_t elapsed = static_cast<uint64_t>(now_ns - first_seen_ns_);
      out.observation_ns =
          elapsed > static_cast<uint64_t>(kLongWindowNs)
              ? static_cast<uint64_t>(kLongWindowNs)
              : elapsed;
    }

    const bool has_short_cutoff = now_ns >= kShortWindowNs;
    const int64_t short_cutoff =
        has_short_cutoff ? now_ns - kShortWindowNs : 0;
    for (std::deque<Sample>::const_iterator it = samples_.begin();
         it != samples_.end(); ++it) {
      AddLong(it->counters, &out);
      if (!has_short_cutoff || it->when_ns > short_cutoff) {
        AddShort(it->counters, &out);
      }
    }

    /** Convert before addition so even near-wrap diagnostic counters cannot
     *  overflow the integer denominator. */
    out.loss_60s_has_data = out.received_60s != 0 || out.lost_60s != 0;
    if (out.loss_60s_has_data) {
      const double denominator = static_cast<double>(out.received_60s) +
                                 static_cast<double>(out.lost_60s);
      out.loss_60s_percent =
          100.0 * static_cast<double>(out.lost_60s) /
          denominator;
    }
    return out;
  }

  bool initialized_;
  std::string broadcast_code_;
  uint64_t connection_generation_;
  int64_t first_seen_ns_;
  int64_t last_update_ns_;
  DashboardCounters previous_;
  std::deque<Sample> samples_;
};

enum DashboardTrend {
  kDashboardTrendActive,
  kDashboardTrendRecovering,
  kDashboardTrendIdle,
  kDashboardTrendUnstable,
  kDashboardTrendWatch,
  kDashboardTrendObserve,
  kDashboardTrendStable
};

/** Current state takes precedence over historical quality.  In particular,
 *  intentionally sleeping/standby lidars show IDLE, not a stale warning. */
struct DashboardLiveSignals {
  bool incident_active;
  bool recovery_active;
  bool intentionally_idle;

  DashboardLiveSignals()
      : incident_active(false),
        recovery_active(false),
        intentionally_idle(false) {}
};

/** Pure classification: no clock, ROS, SDK, global state, or side effects. */
inline DashboardTrend EvaluateTrend(const DashboardWindow &window,
                                    const DashboardLiveSignals &live) {
  if (live.incident_active) {
    return kDashboardTrendActive;
  }
  if (live.recovery_active) {
    return kDashboardTrendRecovering;
  }
  if (live.intentionally_idle) {
    return kDashboardTrendIdle;
  }

  const bool loss_unstable =
      window.loss_60s_has_data && window.loss_60s_percent >= 1.0;
  /** Disconnect is deliberately excluded: a shared four-lidar relay can make
   *  healthy peers disconnect during one group recovery.  Power-request edges
   *  are also excluded because retries/re-offers are actions within an episode;
   *  only the unique power-reached episode count is stability evidence. */
  const bool repeated_same_kind =
      window.handshake_stuck_10m >= 2 || window.power_reached_10m >= 2 ||
      window.fault_10m >= 2 || window.reboot_10m >= 2 ||
      window.mode_fail_10m >= 2;
  if (loss_unstable || repeated_same_kind) {
    return kDashboardTrendUnstable;
  }

  const bool loss_watch =
      window.loss_60s_has_data && window.loss_60s_percent >= 0.1;
  if (window.HasRecentEpisodeOrAction() || window.HandshakeErrors60s() != 0 ||
      window.queue_drops_60s != 0 || loss_watch) {
    return kDashboardTrendWatch;
  }

  if (window.observation_ns <
      static_cast<uint64_t>(DashboardMetrics::kLongWindowNs)) {
    return kDashboardTrendObserve;
  }
  return kDashboardTrendStable;
}

inline const char *DashboardTrendName(DashboardTrend trend) {
  switch (trend) {
    case kDashboardTrendActive:
      return "ACTIVE";
    case kDashboardTrendRecovering:
      return "RECOVERING";
    case kDashboardTrendIdle:
      return "IDLE";
    case kDashboardTrendUnstable:
      return "UNSTABLE";
    case kDashboardTrendWatch:
      return "WATCH";
    case kDashboardTrendObserve:
      return "OBSERVE";
    case kDashboardTrendStable:
      return "STABLE";
  }
  return "?";
}

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER_DASHBOARD_METRICS_H_
