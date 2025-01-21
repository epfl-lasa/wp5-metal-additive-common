#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <boost/bind.hpp>
#include <functional>
#include <memory>
#include <thread>
#include <unordered_map>

#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"

class TaskManager {
public:
  TaskManager(ros::NodeHandle nh, ros::NodeHandle nh_private_);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // This is an independent, dedicated queue for serving the shutdown and
  // kill services. Why?: if we need to stop or kill this node then we would
  // like to do so with certainty; otherwise, since everything runs
  // single-threadedly, our stop/kill signal may be blocked. (An example of this
  // is when calling /enable and then /disable when task_planner_notify_end_
  // is true without launching a task planner: this node will wait for the
  // existence of task planner's service ../execution/join which will make all
  // calls to other services that this node offers impotent)
  ros::CallbackQueue independent_service_queue_;

  // Spins independent_service_queue in a separate thread
  ros::AsyncSpinner async_spinner_;

  // Service servers to power this node on and off
  ros::ServiceServer enable_service_;
  ros::ServiceServer disable_pause_service_;
  ros::ServiceServer disable_stop_service_;
  ros::ServiceServer shutdown_service_;
  ros::ServiceServer kill_service_;

  // Service client for task planner to join when this node's execution is
  // over
  ros::ServiceClient trigger_join_service_client_;

  // A timer that checks whether it's time to modify the state of the
  // functionality that this node provides
  ros::Timer clock_;

  // Available states
  enum States_ { PLAY = 0, PAUSE = 1, STOP = 2, SHUTDOWN = 3, KILL = 4 };
  std::vector<std::vector<bool> > transition_table_;

  enum States_ state_t0_init_ = STOP;
  enum States_ state_t1_init_ = STOP;
  std::atomic<States_> state_t0_;
  std::atomic<States_> state_t1_;
  std::atomic<bool> is_node_alive_;

  // In case of shutdown or kill: the call to notification of the task
  // planner must occur inside the clock callback instead of inside the
  // handleRequest{Kill,Shutdown}; otherwise ROS forces a 5 sec block.
  // If then the call to notification happens inside the callback, the
  // threads running commit*Suicide run on their own, and perhaps kill or
  // shutdown happens before the task planner has been notified. This variable
  // mitigates this.
  bool is_task_planner_notified_;

  // Set this at the end
  bool has_functionality_been_delivered_successfully_;

  // Does this node need to notify the task planner that its functionality
  // has been served? Or is this node running solo?
  bool task_planner_notify_end_;

  // Maps a thread's given name to its native_handle()
  std::unordered_map<std::string, pthread_t> pthread_map_;

  /***************************************************************************
     * @brief Executes periodically. Reads the execution status flags, which are
     * modified from the service server calls coming from the task planner
     * that manages this pkg.
     */
  void clockCallback(const ros::TimerEvent& te);

  /***************************************************************************
     * @brief The final act: calls ros::shutdown()
     */
  void commitNodeSuicide();

  /***************************************************************************
     * @brief The final act: calls system("kill self pid")
     */
  void commitProcessSuicide();

  /***************************************************************************
     * @brief Wraps the functionality that this package provides for use with
     * a task planner
     */
  void entrypointWrapper();

  /***************************************************************************
     * @brief TaskManager: the functionality that the package offers
     */
  void executeTask();

  /***************************************************************************
     * @brief Service servers to start, pause, and stop the functionality that
     * this node offers, and kill the node altogether
     */
  bool handleRequestDisablePause(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool handleRequestDisableStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool handleRequestEnable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool handleRequestShutdown(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool handleRequestKill(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /***************************************************************************
     * @brief Notifies the task planner that manages this pkg that this node has
     * finished its work (or that it was stopped/killed in the process---and its
     * work was forced to finish).
     */
  bool taskPlannerNotifyEnd();

  /***************************************************************************
     * @brief If the entrypoint of your node is not a periodic callback then you
     * may spin up a thread.
     */
  void threadStart(const std::string& tname);
  void threadStop(const std::string& tname);
};
