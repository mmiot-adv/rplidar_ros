#include <thread>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <iostream>
#include <unistd.h>
#include <csignal>
#include "ros/assert.h"
#include "ros/console.h"
#include <sys/types.h>
#include <watchdog.h>

namespace rp
{
RateWatcher::RateWatcher(double target_hz) : RateWatcher(target_hz, 0.1)
{
}
RateWatcher::RateWatcher(double target_hz, double tolerance)
  : RateWatcher(target_hz, tolerance, ros::Duration((3 / tolerance) / target_hz))
{
}
RateWatcher::RateWatcher(double target_hz, double tolerance, ros::Duration window)
  : target_hz(target_hz), tolerance(tolerance), _window(window)
{
  ROS_ASSERT_MSG(tolerance < 1, "tolerance for the watchdog should be smaller than 1");
}
RateWatcher::~RateWatcher()
{
}
void RateWatcher::_delete_old_timestamps()
{
  ros::Time now = ros::Time::now();
  while (!_timestamps.empty() && now - _timestamps.front() > _window)
  {
    _timestamps.pop_front();
  }
}
void RateWatcher::update()
{
  _timestamps.push_back(ros::Time::now());
  _delete_old_timestamps();
}
double RateWatcher::get_rate()
{
  _delete_old_timestamps();
  if (_timestamps.size() < 2)
    return 0;
  return _timestamps.size() / _window.toSec();
}
bool RateWatcher::is_ok()
{
  return get_rate() > target_hz * (1 - tolerance);
}
void RateWatcher::reset()
{
  _timestamps.clear();
}

Watchdog::Watchdog() : is_running(false)
{
  cb_timeout = [] {
    ROS_WARN_STREAM("watchdog: timeout, killing myself (pid: " << getpid() << ")");
    ros::shutdown();
  };
}
Watchdog::Watchdog(std::function<void()> callback) : is_running(false), cb_timeout(callback)
{
}
Watchdog::~Watchdog()
{
  stop();
}
void Watchdog::start(ros::Duration reset_timeout, double target_hz, double tolerance)
{
  std::unique_lock<std::mutex> lock(mtx);
  if (is_running)
    return;
  ROS_INFO("starting watchdog with timeout: %.1fs and target frequency: %.1fHz", reset_timeout.toSec(), target_hz);

  last_ok_time = ros::Time::now();
  this->reset_timeout = reset_timeout;
  rate_watcher = std::make_shared<RateWatcher>(target_hz, tolerance);
  is_running = true;
  wdt_thread = std::thread(&Watchdog::loop, this);
}

void Watchdog::stop()
{
  std::unique_lock<std::mutex> lock(mtx);
  if (!is_running)
    return;

  is_running = false;
  cv_stop.notify_all();
  lock.unlock();
  wdt_thread.join();
}

void Watchdog::update()
{
  std::unique_lock<std::mutex> lock(mtx);
  rate_watcher->update();
  if (rate_watcher->is_ok())
  {
    last_ok_time = ros::Time::now();
    cv_stop.notify_all();
  }
  else
  {
    ROS_WARN_THROTTLE(1, "watchdog: rate is too low: %.1fHz/%.1fHz", rate_watcher->get_rate(),
                      rate_watcher->target_hz);
  }
}

void Watchdog::loop()
{
  while (is_running)
  {
    std::unique_lock<std::mutex> lock(mtx);
    if (cv_stop.wait_for(lock, std::chrono::milliseconds(static_cast<int64_t>(reset_timeout.toSec() * 1000))) ==
        std::cv_status::timeout)
    {
      if (cb_timeout != nullptr)
      {
        is_running = false;
        cb_timeout();
      }
    }
  }
}

}  // namespace rp
