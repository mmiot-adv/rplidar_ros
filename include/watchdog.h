#ifndef _watchdog_h
#define _watchdog_h

#include <deque>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <iostream>
#include <functional>

#include <ros/ros.h>

namespace rp
{
// @brief a class to keep track of the update rate
class RateWatcher
{
private:
  // buffer to store the timestamps
  std::deque<ros::Time> _timestamps;
  // the duration to keep the timestamps in the buffer.
  // defaults to ((3 / tolerance) / target_hz)
  ros::Duration _window;
  // @brief delete the timestamps that are older than (now - window)
  void _delete_old_timestamps();

public:
  RateWatcher(double target_hz);
  RateWatcher(double target_hz, double tolerance);
  RateWatcher(double target_hz, double tolerance, ros::Duration window);
  ~RateWatcher();
  // @brief call this function to add a new timestamp to the buffer
  void update();
  // @brief return the current rate
  double get_rate();
  // @brief return true if the update rate is higher than target_hz * (1 - tolerance)
  bool is_ok();
  // @brief resets the timestamp buffer
  void reset();
  // the target update rate
  double target_hz;
  // the tolerance of the update rate, if lower than target_hz * (1 - tolerance), the rate is considered to be too low
  double tolerance;
};

// @brief a watchdog class to monitor the update rate and act accordingly
class Watchdog
{
public:
  Watchdog();
  Watchdog(std::function<void()> callback);
  ~Watchdog();
  /**
   * @brief starts the watchdog. Will do nothing if the watchdog is already running.
   *
   * @param reset_timeout the watchdog will be triggered if the last refresh time is older than this timeout
   * @param target_hz the target update rate
   * @param tolerance the watchdog will be triggered if the update rate is lower than (1 - tolerance) * target_hz
   */
  void start(ros::Duration reset_timeout, double target_hz, double tolerance = 0.1);
  // @brief stops the watchdog. Will do nothing if the watchdog is not running.
  void stop();
  // @brief call this function to update the watchdog with a new timestamp
  void update();

private:
  // if last_ok_time is older than reset_timeout, the watchdog will be triggered and the callback function will be
  // called
  ros::Duration reset_timeout;
  std::shared_ptr<RateWatcher> rate_watcher;
  // if the watchdog is running
  std::atomic<bool> is_running;
  // the thread to run the watchdog
  std::thread wdt_thread;
  // the callback function to be called when watchdog is triggered, default is to call ros::shutdown()
  std::function<void()> cb_timeout;
  // the global mutex
  std::mutex mtx;
  // the last time the watchdog is considered to be ok
  ros::Time last_ok_time;
  // the condition variable to stop the watchdog thread
  std::condition_variable cv_stop;
  // the main loop of the watchdog thread
  void loop();
};

}  // namespace rp

#endif /* _watchdog_h */
