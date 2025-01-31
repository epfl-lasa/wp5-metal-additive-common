#include "TaskManager.hpp"

#include <chrono>
#include <string>
#include <thread>

#include "TaskFSM.h"
#include "TaskFactory.h"

/*******************************************************************************
*/
TaskManager::TaskManager(ros::NodeHandle nh, ros::NodeHandle nh_private) :
    nh_(nh),
    nh_private_(nh_private),
    state_t0_(state_t0_init_),
    state_t1_(state_t1_init_),
    is_node_alive_(true),
    is_task_planner_notified_(false),
    has_functionality_been_delivered_successfully_(false),
    async_spinner_(1, &independent_service_queue_) {
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  // Task planner's requirements
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  // Call this service to start functionality
  enable_service_ =
      nh_.advertiseService(ros::this_node::getName() + "/execution/enable", &TaskManager::handleRequestEnable, this);

  // Call this service to pause functionality
  disable_pause_service_ = nh_.advertiseService(
      ros::this_node::getName() + "/execution/disable/pause", &TaskManager::handleRequestDisablePause, this);

  // Call this service to stop functionality
  disable_stop_service_ = nh_.advertiseService(
      ros::this_node::getName() + "/execution/disable/stop", &TaskManager::handleRequestDisableStop, this);

  // The details of the shutdown service. It runs in a separate thread so that
  // it can override normal execution
  ros::AdvertiseServiceOptions shutdown_service_options = ros::AdvertiseServiceOptions::create<std_srvs::Trigger>(
      ros::this_node::getName() + "/execution/halt/shutdown_node",
      boost::bind(&TaskManager::handleRequestShutdown, this, _1, _2),
      ros::VoidPtr(),
      &independent_service_queue_);

  // The details of the kill service. It runs in a separate thread so that
  // it can override normal execution
  ros::AdvertiseServiceOptions kill_service_options = ros::AdvertiseServiceOptions::create<std_srvs::Trigger>(
      ros::this_node::getName() + "/execution/halt/kill",
      boost::bind(&TaskManager::handleRequestKill, this, _1, _2),
      ros::VoidPtr(),
      &independent_service_queue_);

  // Call this service to shutdown this node
  shutdown_service_ = nh_.advertiseService(shutdown_service_options);

  // Call this service to kill the process that runs this node
  kill_service_ = nh_.advertiseService(kill_service_options);

  // This node may now wait for service requests to its {shutdown,kill} service
  async_spinner_.start();

  // This service should be called by code in this node in order to signify
  // the end of execution of this node to the task planner that enabled it
  trigger_join_service_client_ = nh_.serviceClient<std_srvs::SetBool>(ros::this_node::getName() + "/execution/join");

  // A timer that checks whether it's time to modify the state of the
  // functionality that this node provides
  clock_ = nh_.createTimer(ros::Duration(1), &TaskManager::clockCallback, this);

  // ---------------------------------------------------------------------------
  // Does this node need to notify the task planner that its functionality
  // has been served? Or is this node running solo?
  if (!nh_private_.getParam("task_planner_notify_end", task_planner_notify_end_)) {
    ROS_WARN("No task_planner_notify_end param found; resorting to default");
    task_planner_notify_end_ = false;
  }

  // ---------------------------------------------------------------------------
  // Transitions between states
  // There are 5 available states in States_
  size_t w = 5;
  size_t h = w;

  for (unsigned int i = 0; i < w; i++) {
    std::vector<bool> tmp;
    for (unsigned int j = 0; j < h; j++) tmp.push_back(false);

    transition_table_.push_back(tmp);
  }

  // transition_table_[PLAY][PLAY]    = false;
  transition_table_[PLAY][PAUSE] = true;
  transition_table_[PLAY][STOP] = true;
  transition_table_[PLAY][SHUTDOWN] = true;
  transition_table_[PLAY][KILL] = true;
  //----------------------------------------------------
  transition_table_[PAUSE][PLAY] = true;
  // transition_table_[PAUSE][PAUSE]  = false;
  transition_table_[PAUSE][STOP] = true;
  transition_table_[PAUSE][SHUTDOWN] = true;
  transition_table_[PAUSE][KILL] = true;
  //-----------------------------------------------------
  transition_table_[STOP][PLAY] = true;
  // transition_table_[STOP][PAUSE]   = false;
  // transition_table_[STOP][STOP]    = false;
  transition_table_[STOP][SHUTDOWN] = true;
  transition_table_[STOP][KILL] = true;
  //-----------------------------------------------------
  // transition_table_[SHUTDOWN][*]   = false;
  //-----------------------------------------------------
  // transition_table_[KILL][*]       = false;
  //-----------------------------------------------------
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
}

/*******************************************************************************
 * @brief Executes periodically.
 */
void TaskManager::clockCallback(const ros::TimerEvent& te) {
  // ---------------------------------------------------------------------------
  if (state_t0_.load() != state_t1_.load()) {
    // Transition to state if transition is allowed
    if (transition_table_[state_t0_.load()][state_t1_.load()] == true) {
      ROS_INFO("EXECUTING TRANSITION: %d -> %d", state_t0_.load(), state_t1_.load());

      // Store new state to old
      state_t0_.store(state_t1_.load());

      // -------------------------------------
      if (state_t1_.load() == PLAY) {
        ROS_INFO("Triggering execution PLAY");
        threadStart("shotcreting_thread");
      }
      // -------------------------------------
      if (state_t1_.load() == PAUSE) {
        ROS_INFO("Triggering execution PAUSE");
        threadStop("shotcreting_thread");
      }
      // -------------------------------------
      if (state_t1_.load() == STOP) {
        ROS_INFO("Triggering execution STOP");
        threadStop("shotcreting_thread");

        // Notify that TaskManager has ended unsuccessfully first (otherwise the
        // task planner that called this package cannot join it after it was
        // stopped, which means that it will block)
        if (task_planner_notify_end_ == true)
          taskPlannerNotifyEnd();
      }
      // -------------------------------------
      if (state_t1_.load() == SHUTDOWN) {
        // Handled in own service callback. Otherwise functions called in this
        // scope might block SHUTDOWN

        // Notify that TaskManager has ended unsuccessfully first (otherwise the
        // task planner that called this package cannot join it after it was
        // stopped, which means that it will block)
        if (task_planner_notify_end_ == true)
          is_task_planner_notified_ = taskPlannerNotifyEnd();
      }
      // -------------------------------------
      if (state_t1_.load() == KILL) {
        // Handled in own service callback. Otherwise functions called in this
        // scope might block KILL

        // Notify that TaskManager has ended unsuccessfully first (otherwise the
        // task planner that called this package cannot join it after it was
        // stopped, which means that it will block)
        if (task_planner_notify_end_ == true)
          is_task_planner_notified_ = taskPlannerNotifyEnd();
      }
    } else {
      ROS_INFO("TRANSITION: %d -> %d NOT ALLOWED", state_t0_.load(), state_t1_.load());

      // Store old state to new
      state_t1_.store(state_t0_.load());
    }
  }
  // ---------------------------------------------------------------------------
}

/*******************************************************************************
 * @brief The final act: calls ros::shutdown()
 */
void TaskManager::commitNodeSuicide() {
  if (task_planner_notify_end_ == true) {
    while (is_task_planner_notified_ == false) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ROS_INFO("Triggering node SHUTDOWN");
  ros::shutdown();
}

/*******************************************************************************
 * @brief The final act: calls system("kill self pid")
 */
void TaskManager::commitProcessSuicide() {
  if (task_planner_notify_end_ == true) {
    while (is_task_planner_notified_ == false) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ROS_INFO("Triggering node KILL");

  pid_t pid = getpid();
  std::string s = "kill -9 " + std::to_string(pid);
  int ret = system(s.c_str());
}

/*******************************************************************************
 * @brief Wraps the functionality that this package provides for use with
 * a task planner
 */
void TaskManager::entrypointWrapper() {
  executeTask();

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  // The functionality has been delivered successfully, or maybe it hasn't
  has_functionality_been_delivered_successfully_ = true;

  // Execution has ended; return to initial state
  state_t1_.store(state_t1_init_);
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
}

/*******************************************************************************
 * @brief TaskManager: the functionality that the package offers
 */
void TaskManager::executeTask() {
  // Check for the taskType parameter
  std::string taskType = "";
  ros::param::get("~task_type", taskType);

  // Create a unique pointer for the instance of TaskFSM
  ROS_INFO("[MainTask] - Creating Task - %s", taskType.c_str());
  std::shared_ptr<ITaskBase> task = TaskFactory::createTask(taskType, nh_, std::string("robot_task.yaml"));

  taskFsm_ internalFSM_(task, nh_);

  // Initialize and start the FSM
  internalFSM_.start();
  internalFSM_.process_event(Start());
  internalFSM_.stop();

  ROS_INFO("DONE TaskManager.");
}

/*******************************************************************************
 * @brief Service server to pause the functionality that this node offers
 */
bool TaskManager::handleRequestDisablePause(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  ROS_INFO("Requesting execution PAUSE");
  state_t1_.store(PAUSE);

  res.message = "SUCCESS";
  res.success = true;
  return res.success;
}

/*******************************************************************************
 * @brief Service server to stop the functionality that this node offers
 * (pause + return to task planner)
 */
bool TaskManager::handleRequestDisableStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  ROS_INFO("Requesting execution STOP");
  state_t1_.store(STOP);

  res.message = "SUCCESS";
  res.success = true;
  return res.success;
}

/*******************************************************************************
 * @brief Service server to start the functionality that this node offers
 */
bool TaskManager::handleRequestEnable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  ROS_INFO("Requesting execution PLAY");
  state_t1_.store(PLAY);

  res.message = "SUCCESS";
  res.success = true;
  return res.success;
}

/*******************************************************************************
 * @brief Service server to kill the process that runs the node
 */
bool TaskManager::handleRequestKill(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  ROS_WARN("Requesting node KILL");

  state_t1_.store(KILL);
  is_node_alive_.store(false);

  // Spin a new thread so that the service has time to respond back
  std::thread t = std::thread(&TaskManager::commitProcessSuicide, this);
  t.detach();

  res.message = "SUCCESS";
  res.success = true;
  return res.success;
}

/*******************************************************************************
 * @brief Service server to shutdown the node altogether
 * https://answers.ros.org/question/294069/shutdown-a-node-from-another-node-in-ros-cpp/
 */
bool TaskManager::handleRequestShutdown(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  ROS_WARN("Requesting node SHUTDOWN");

  state_t1_.store(SHUTDOWN);
  is_node_alive_.store(SHUTDOWN);

  // Spin a new thread so that the service has time to respond back
  std::thread t = std::thread(&TaskManager::commitNodeSuicide, this);
  t.detach();

  res.message = "SUCCESS";
  res.success = true;
  return res.success;
}

/*******************************************************************************
 * @brief Notifies the task planner that manages this pkg that this node has
 * finished its work (or that it was stopped/shut down in the process---and its
 * work was forced to finish).
 */
bool TaskManager::taskPlannerNotifyEnd() {
  while (!ros::service::waitForService(trigger_join_service_client_.getService(), ros::Duration(1.0))) {
    if (!ros::ok()) {
      ROS_ERROR("Client of service %s interrupted while waiting for service to appear",
                trigger_join_service_client_.getService().c_str());
      return false;
    }
    ROS_WARN("Waiting for service %s to appear...", trigger_join_service_client_.getService().c_str());
  }
  // service appeared

  // Craft request
  std_srvs::SetBool srv;
  srv.request.data = has_functionality_been_delivered_successfully_;

  // Notify the task planner that this package ended work
  // and continue with execution
  while (!trigger_join_service_client_.call(srv)) {
    ROS_ERROR("Failed to call join service");
    ros::Duration(0.5).sleep();
  }

  if (!ros::ok()) {
    ROS_ERROR("Program canceled");
    return false;
  }

  ROS_INFO("Will now join main task planner caller ...");
  return true;
}

/*******************************************************************************
 * @brief If the entrypoint of your node is not a periodic callback then you
 * may spin up a thread
 */
void TaskManager::threadStart(const std::string& tname) {
  std::thread t = std::thread(&TaskManager::entrypointWrapper, this);
  pthread_map_[tname] = t.native_handle();
  t.detach();

  ROS_INFO("Thread %s is alive", tname.c_str());
}

/*******************************************************************************
 * @brief https://github.com/bo-yang/terminate_cpp_thread/blob/master/kill_cpp_thread.cc
 */
void TaskManager::threadStop(const std::string& tname) {
  std::unordered_map<std::string, pthread_t>::const_iterator it = pthread_map_.find(tname);

  if (it != pthread_map_.end()) {
    pthread_cancel(it->second);
    pthread_map_.erase(tname);

    ROS_INFO("Thread %s is rightfully dead", tname.c_str());
  }
}
