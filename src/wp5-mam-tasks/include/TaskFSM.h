#pragma once

// MSM back-end
#include <boost/msm/back/state_machine.hpp>

// MSM front-end
#include <boost/mp11/mpl.hpp>
#include <boost/msm/front/euml/euml.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

// Other
#include <iostream>
#include <memory>
#include <string>

#include "ITaskBase.h"

namespace msm = boost::msm;
namespace msmf = msm::front;
namespace mp11 = boost::mp11;

// List of FSM events
class Initialized {};
class PathComputed {};
class Start {};
class Finished {};

class ErrorTrigger {};
class ErrorAcknowledgement {};

/**
 * @class TaskFSM
 * @brief Represents a finite state machine for task management.
 *
 * The TaskFSM class is a state machine that manages the execution of tasks.
 * It defines various states and transitions between them based on events.
 * The class provides methods to set and retrieve the current state, as well
 * as perform actions and guard conditions associated with each state transition.
 */
class TaskFSM : public msmf::state_machine_def<TaskFSM> {
private:
  // List of FSM states
  class Initializing;
  class Planning;
  class Ready;
  class Executing;
  class Homing;
  class Exit;

  class AllOk;
  class ErrorMode;

protected:
  bool exit_ = true;
  bool homed_ = false;
  bool ready_ = false;

  std::string error_ = "";
  std::shared_ptr<ITaskBase> currentTask_;

public:
  TaskFSM(std::shared_ptr<ITaskBase> task) : currentTask_(task) { std::cout << "Inside TaskFSM : " << std::endl; };

  bool getExit() { return exit_; }
  bool getHomed() { return homed_; }
  bool getReady() { return ready_; }
  std::string getError() { return error_; }
  std::shared_ptr<ITaskBase> getCurrentTask() { return currentTask_; }

  void setExit(bool exit) { exit_ = exit; }
  void setHome(bool homed) { homed_ = homed; }
  void setReady(bool ready) { ready_ = ready; }
  void setError(std::string error) { error_ = error; }

  /**
   * @brief Action functor class for the goWorkingPosition event in the TaskFSM.
   */
  class goWorkingPosition {
  public:
    /**
     * @brief Operator overload for handling the goWorkingPosition event.
     *
     * This operator is called when the goWorkingPosition event is triggered in the FSM.
     * It checks if the current task can go to the working position and updates the FSM accordingly.
     * If the task is successful, it sets the FSM to ready state. Otherwise, it sets an error message
     * and triggers the ErrorTrigger event in the FSM.
     *
     * @param evt The event object.
     * @param fsm The finite state machine object.
     * @param src The source state object.
     * @param tgt The target state object.
     */
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const& evt, FSM& fsm, SourceState& src, TargetState& tgt) {
      bool feedback = fsm.getCurrentTask()->goWorkingPosition();

      if (feedback) {
        fsm.setReady(true);
      } else {
        fsm.setError("Error: Task goWorkingPosition failed");
        fsm.process_event(ErrorTrigger());
      }
    }
  };

  /**
   * @brief Guard condition functor used to check if the FSM is ready to transition from the source state to the target
   * state.
   */
  class isReady {
  public:
    /**
     * @brief Operator overload for the functor.
     *
     * This operator is called when the functor is invoked. It takes the current event, FSM object, source state, and
     * target state as arguments and returns a boolean value indicating if the FSM is ready to transition.
     *
     * @param evt The current event.
     * @param fsm The FSM object.
     * @param src The source state.
     * @param tgt The target state.
     * @return True if the FSM is ready to transition, false otherwise.
     */
    template <class EVT, class FSM, class SourceState, class TargetState>
    bool operator()(EVT const& evt, FSM& fsm, SourceState& src, TargetState& tgt) {
      return fsm.getReady();
    }
  };

  /**
   * @brief Guard condition functor class to check if the FSM is in the "homed" state.
   */
  class isHomed {
  public:
    /**
     * @brief Operator overload to check if the FSM is in the "homed" state.
     *
     * This operator is called when evaluating the guard condition in a Boost.Statechart transition.
     * It takes the event, FSM, source state, and target state as arguments and returns a boolean value indicating
     * whether the FSM is in the "homed" state or not.
     *
     * @param evt The event object.
     * @param fsm The FSM object.
     * @param src The source state object.
     * @param tgt The target state object.
     * @return True if the FSM is in the "homed" state, false otherwise.
     */
    template <class EVT, class FSM, class SourceState, class TargetState>
    bool operator()(EVT const& evt, FSM& fsm, SourceState& src, TargetState& tgt) {
      return fsm.getHomed();
    }
  };

  /**
   * @brief Guard condition functor used to check if the FSM is exiting.
   */
  class isExiting {
  public:
    /**
     * @brief Checks if the FSM is exiting.
     *
     * This function is called by the FSM during state transitions to determine if the FSM should exit.
     *
     * @param evt The event object.
     * @param fsm The FSM object.
     * @param src The source state object.
     * @param tgt The target state object.
     * @return true if the FSM is exiting, false otherwise.
     */
    template <class EVT, class FSM, class SourceState, class TargetState>
    bool operator()(EVT const& evt, FSM& fsm, SourceState& src, TargetState& tgt) {
      return fsm.getExit();
    }
  };

  typedef mp11::mp_list<AllOk, Initializing> initial_state;

  // Each row correspond to : Start, Event, Next, Action, Guard
  using transition_table = mp11::mp_list<
      // Initializing -----------------------------------------
      msmf::Row<Initializing, Initialized, Planning, msmf::none, msmf::none>,

      // Planning ---------------------------------------------
      msmf::Row<Planning, PathComputed, Ready, goWorkingPosition, msmf::none>,

      // Ready ------------------------------------------------
      msmf::Row<Ready, Start, Executing, msmf::none, isReady>,

      // Executing --------------------------------------------
      msmf::Row<Executing, Finished, Ready, goWorkingPosition, msmf::euml::Not_<isExiting>>,
      msmf::Row<Executing, Finished, Homing, msmf::none, isExiting>,

      // Homing -----------------------------------------------
      msmf::Row<Homing, msmf::none, Exit, msmf::none, isHomed>,

      // Error ------------------------------------------------
      msmf::Row<AllOk, ErrorTrigger, ErrorMode, msmf::none, msmf::none>,
      msmf::Row<ErrorMode, ErrorAcknowledgement, AllOk, msmf::none, msmf::none>>;
};

typedef msm::back::state_machine<TaskFSM> taskFsm_;

/**
 * @brief The Initializing state of the TaskFSM class.
 */
class TaskFSM::Initializing : public msmf::state<> {
public:
  /**
   * @brief Called when entering the "Initializing" state.
   *
   * This state is responsible for initializing the current task. It calls the `initialize()` method of the current
   * task and processes the appropriate events based on the feedback received from the task initialization.
   * If the task initialization is successful, it transitions to the Initialized state. Otherwise, it sets an error
   * message and transitions to the ErrorTrigger state.
   *
   * @tparam Event The type of the event.
   * @tparam FSM The type of the state machine.
   * @param event The event that triggered the state transition.
   * @param fsm The state machine instance.
   */
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    bool feedback = fsm.getCurrentTask()->initialize();

    if (feedback) {
      fsm.process_event(Initialized());
    } else {
      fsm.setError("Error: Task initialization failed");
      fsm.process_event(ErrorTrigger());
    }
  }
};

/**
 * @brief The Planning state of the TaskFSM class.
 */
class TaskFSM::Planning : public msmf::state<> {
public:
  /**
   * @brief Called when entering the "Planning" state.
   *
   * This state is responsible for computing the path for the current task.
   * If the path computation is successful, it transitions to the PathComputed state.
   * If the path computation fails, it sets an error message and transitions to the ErrorTrigger state.
   *
   * @tparam Event The type of the event.
   * @tparam FSM The type of the state machine.
   * @param event The event that triggered the state transition.
   * @param fsm The state machine instance.
   */
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    bool feedback = fsm.getCurrentTask()->computePath();

    if (feedback) {
      fsm.process_event(PathComputed());
    } else {
      fsm.setError("Error: Path computation failed");
      fsm.process_event(ErrorTrigger());
    }
  }
};

/**
 * @brief The Planning state of the TaskFSM class.
 */
class TaskFSM::Ready : public msmf::state<> {
public:
  /**
   * @brief Called when entering the Ready state.
   *
   * This state is responsible for managing the share mode of the TaskFSM.
   * It provides methods to get and set the share mode.
   *
   * @tparam Event The type of the event.
   * @tparam FSM The type of the finite state machine.
   * @param event The event that triggered the transition.
   * @param fsm The finite state machine.
   */
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: TaskFSM - Ready" << std::endl;
  }

  /**
     * @brief Called when exiting the Ready state.
     *
     * @tparam Event The type of the event.
     * @tparam FSM The type of the finite state machine.
     * @param event The event that triggered the transition.
     * @param fsm The finite state machine.
     */
  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: TaskFSM - Ready" << std::endl;
  }
};

/**
  * @brief The Executing state of the TaskFSM class.
  */
class TaskFSM::Executing : public msmf::state<> {
public:
  /**
   * @brief Called when entering the "Executing" state.
   *
   * This state represents the execution of a task in the finite state machine.
   * When entering this state, the `on_entry` function is called, which executes the current task.
   * If the task execution is successful, the state machine transitions to the `Finished` state.
   * If the task execution fails, an error message is set and the state machine transitions to the `ErrorTrigger` state.
   *
   * @tparam Event The type of the event.
   * @tparam FSM The type of the state machine.
   * @param event The event that triggered the state transition.
   * @param fsm The state machine instance.
   */
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    bool feedback = fsm.getCurrentTask()->execute();

    if (feedback) {
      fsm.process_event(Finished());
    } else {
      fsm.setError("Error: Task execution failed");
      fsm.process_event(ErrorTrigger());
    }
  }
};

/**
 * @brief The Homing state of the TaskFSM class.
 */
class TaskFSM::Homing : public msmf::state<> {
public:
  /**
   * @brief Called when entering the "Homing" state.
   *
   * This function executes the `goHomingPosition()` function of the current task and checks the feedback. If the
   * feedback indicates a failure, it sets an error message and triggers the `ErrorTrigger` event to transition to
   * the error state.
   *
   * @tparam Event The type of the event.
   * @tparam FSM The type of the state machine.
   * @param event The event that triggered the state transition.
   * @param fsm The state machine instance.
   */
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    bool feedback = fsm.getCurrentTask()->goHomingPosition();

    if (feedback) {
      fsm.setHome(true);
    } else {
      fsm.setError("Error: Task homing failed");
      fsm.process_event(ErrorTrigger());
    }
  }

  /**
   * @brief Called when exiting the "Homing" state.
   *
   * This function is called when transitioning out of the "Homing" state. It simply prints a message indicating that
   * the state is being left.
   *
   * @tparam Event The type of the event.
   * @tparam FSM The type of the state machine.
   * @param event The event that triggered the state transition.
   * @param fsm The state machine instance.
   */
  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: TaskFSM - Homing" << std::endl;
  }
};

/**
  * @brief Represents the Exit state of the TaskFSM.
  */
class TaskFSM::Exit : public msm::front::terminate_state<> {
public:
  /**
   * @brief Called when entering the Exit state.
   *
   * @tparam Event The type of the event.
   * @tparam FSM The type of the finite state machine.
   * @param event The event that triggered the state transition.
   * @param fsm The finite state machine instance.
   */
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: TaskFSM - Exit" << std::endl;

    ros::shutdown(); // Signal ROS to shut down
  }

  /**
   * @brief Called when exiting the Exit state.
   *
   * @tparam Event The type of the event.
   * @tparam FSM The type of the finite state machine.
   * @param event The event that triggered the state transition.
   * @param fsm The finite state machine instance.
   */
  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: TaskFSM - Exit" << std::endl;
  }
};

/**
 * @brief Represents the AllOk state of the TaskFSM class.
 */
class TaskFSM::AllOk : public msm::front::state<> {
public:
  /**
   * @brief Called when entering the AllOk state.
   *
   * @tparam Event The type of the event triggering the transition.
   * @tparam FSM The type of the finite state machine.
   * @param event The event triggering the transition.
   * @param fsm The finite state machine.
   */
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&) {
    std::cout << "starting: AllOk" << std::endl;
  }

  /**
   * @brief Called when exiting the AllOk state.
   *
   * @tparam Event The type of the event triggering the transition.
   * @tparam FSM The type of the finite state machine.
   * @param event The event triggering the transition.
   * @param fsm The finite state machine.
   */
  template <class Event, class FSM>
  void on_exit(Event const&, FSM&) {
    std::cout << "finishing: AllOk" << std::endl;
  }
};

/**
 * @brief Represents the ErrorMode state of the TaskFSM class.
 */
class TaskFSM::ErrorMode : public msm::front::interrupt_state<ErrorAcknowledgement> {
public:
  /**
   * @brief Called when entering the ErrorMode state.
   *
   * @tparam Event The type of the event.
   * @tparam FSM The type of the finite state machine.
   * @param event The event that triggered the state transition.
   * @param fsm The finite state machine instance.
   */
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "starting: ErrorMode" << std::endl;
  }

  /**
   * @brief Called when exiting the ErrorMode state.
   *
   * @tparam Event The type of the event.
   * @tparam FSM The type of the finite state machine.
   * @param event The event that triggered the state transition.
   * @param fsm The finite state machine instance.
   */
  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "finishing: ErrorMode" << std::endl;
  }
};
