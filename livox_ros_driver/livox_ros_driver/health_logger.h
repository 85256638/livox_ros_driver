//
// Lightweight, thread-safe CSV health logger for unattended long-running
// deployments (e.g. multi-lidar at a steel-mill ladle hot-repair station).
//
// Two append-only streams, rotated daily by the date in the filename:
//   * livox_events_YYYY-MM-DD.csv   -- one row the INSTANT something changes
//       (health-bit change, disconnect, reconnect, auto-reboot). Edge-triggered,
//       so it never misses a transient fault the way a periodic snapshot would.
//   * livox_snapshot_YYYY-MM-DD.csv -- one row per lidar every N seconds,
//       carrying the CUMULATIVE packet counters. Differencing two snapshots
//       gives the loss/recv totals for that interval (gap-free trend, good for
//       correlating packet loss with mill activity / EMI over hours-days).
//
// Disabled by default. Opened lazily per write (so daily rotation is automatic
// and no file handle is held open 24/7). A write failure warns once and
// disables logging so it can never take down the driver.
//
#ifndef LIVOX_ROS_DRIVER_HEALTH_LOGGER_H_
#define LIVOX_ROS_DRIVER_HEALTH_LOGGER_H_

#include <cstdint>
#include <ctime>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <string>

namespace livox_ros {

class HealthLogger {
 public:
  static HealthLogger &Get() {
    static HealthLogger inst;
    return inst;
  }

  void Enable(const std::string &dir, int snapshot_period_s) {
    std::lock_guard<std::mutex> lk(mu_);
    dir_ = dir;
    snapshot_period_s_ = (snapshot_period_s > 0) ? snapshot_period_s : 600;
    enabled_ = true;
  }
  bool enabled() const { return enabled_; }
  int snapshot_period_s() const { return snapshot_period_s_; }

  /** Append one event row, called the instant the event happens. `detail` must
   *  not contain commas (it is the last CSV field). */
  void LogEvent(int handle, const char *bcode, const char *event,
                const std::string &detail) {
    if (!enabled_) {
      return;
    }
    char ts[24];
    NowStamp(ts, sizeof(ts));
    char row[384];
    snprintf(row, sizeof(row), "%s,%d,%s,%s,%s", ts, handle,
             (bcode && bcode[0]) ? bcode : "?", event, detail.c_str());
    Write("events", "wall_time,handle,bcode,event,detail", row);
  }

  /** Persist one confirmed publication-plane outage. Recovery confirmation is
   * written now, while duration deliberately ends at first_data so the three
   * verification seconds do not inflate the outage. */
  void LogPointCloudRecovery(int handle, const char *bcode,
                             int64_t lost_wall_ns,
                             int64_t first_data_wall_ns,
                             int64_t confirmed_wall_ns,
                             int64_t duration_ns) {
    if (!enabled_) {
      return;
    }
    char lost[32];
    char first[32];
    char confirmed[32];
    FormatWallNs(lost_wall_ns, lost, sizeof(lost));
    FormatWallNs(first_data_wall_ns, first, sizeof(first));
    FormatWallNs(confirmed_wall_ns, confirmed, sizeof(confirmed));
    char detail[256];
    snprintf(detail, sizeof(detail),
             "duration=%.3fs; lost_at=%s; first_data_returned=%s; "
             "confirmed_healthy=%s",
             duration_ns > 0 ? duration_ns / 1000000000.0 : 0.0, lost,
             first, confirmed);
    LogEvent(handle, bcode, "POINTCLOUD_RECOVERED", detail);
  }

  /** Append one snapshot row (per lidar, every snapshot_period_s). Carries
   *  cumulative counters so consecutive rows difference into per-interval
   *  totals without missing anything in between. */
  void LogSnapshot(int handle, const char *bcode, const char *state,
                   const char *temp, const char *fan, const char *motor,
                   unsigned dirty, const char *system,
                   unsigned long long recv_total, unsigned long long loss_total,
                   unsigned long long drop_total, double loss_pct,
                   unsigned disc) {
    if (!enabled_) {
      return;
    }
    char ts[24];
    NowStamp(ts, sizeof(ts));
    char row[384];
    snprintf(row, sizeof(row),
             "%s,%d,%s,%s,%s,%s,%s,%u,%s,%llu,%llu,%llu,%.3f,%u", ts, handle,
             (bcode && bcode[0]) ? bcode : "?", state, temp, fan, motor, dirty,
             system, recv_total, loss_total, drop_total, loss_pct, disc);
    Write("snapshot",
          "wall_time,handle,bcode,state,temp,fan,motor,dirty,system,"
          "recv_total,loss_total,drop_total,loss_pct,disc",
          row);
  }

 private:
  HealthLogger() = default;

  static void NowStamp(char *buf, size_t n) {
    time_t t = time(nullptr);
    struct tm tmv;
    localtime_r(&t, &tmv);
    strftime(buf, n, "%Y-%m-%d %H:%M:%S", &tmv);
  }

  static void FormatWallNs(int64_t wall_ns, char *buf, size_t n) {
    if (buf == nullptr || n == 0) {
      return;
    }
    if (wall_ns <= 0) {
      snprintf(buf, n, "--");
      return;
    }
    time_t seconds = static_cast<time_t>(wall_ns / 1000000000LL);
    const long long millis = (wall_ns % 1000000000LL) / 1000000LL;
    struct tm tmv;
    localtime_r(&seconds, &tmv);
    char base[24];
    strftime(base, sizeof(base), "%Y-%m-%d %H:%M:%S", &tmv);
    snprintf(buf, n, "%s.%03lld", base, millis);
  }

  void Write(const char *kind, const char *header, const char *row) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!enabled_) {
      return;
    }
    char date[16];
    time_t t = time(nullptr);
    struct tm tmv;
    localtime_r(&t, &tmv);
    strftime(date, sizeof(date), "%Y-%m-%d", &tmv);
    std::string path = (dir_.empty() ? std::string(".") : dir_) + "/livox_" +
                       kind + "_" + date + ".csv";
    bool need_header = !std::ifstream(path.c_str()).good();
    std::ofstream f(path.c_str(), std::ios::app);
    if (!f.is_open()) {
      if (!warned_) {
        fprintf(stderr,
                "[HealthLog] cannot write %s -- health logging disabled\n",
                path.c_str());
        warned_ = true;
      }
      enabled_ = false;
      return;
    }
    if (need_header) {
      f << header << "\n";
    }
    f << row << "\n";
  }

  std::mutex mu_;
  bool enabled_ = false;
  bool warned_ = false;
  std::string dir_;
  int snapshot_period_s_ = 600;
};

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER_HEALTH_LOGGER_H_
