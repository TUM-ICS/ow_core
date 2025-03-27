/*! \file
 *
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author J. Rogelio Guadarrama-Olvera
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 14.02.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#ifndef OPEN_WALKER_FLAGS_H
#define OPEN_WALKER_FLAGS_H

#include <sstream>
#include <ow_core/types/foot_id.h>
#include <ow_msgs/Flags.h>

/*!
 * \brief Open Walker core classes. Types and tools required to implement all
 * the modules.
 */
namespace ow_core
{

  /*!
  * \brief The Flags class.
  *
  *  This class is a container for the system flags.
  */
  class Flags
  {
  public:

    /**
     * @brief The State of the robot
     * 
     */
    enum State
    {
      IDLE = 0,
      HOMEING,
      STANCE,
      WALKING
    };

    /**
     * @brief Substate of the State
     * 
     */
    enum WalkingPhase
    {
      IDLE_PHASE = 0,
      DOUBLE_SUPPORT = 1000,
      SINGLE_SUPPORT
    };

    /**
     * @brief Substate of the WalkingPhase
     * 
     */
    enum Quater
    {
      IDLE_QUATER = 0,
      DOUBLE_SUPPORT_1 = 2000,  //!< double support below 1/4 time interval
      DOUBLE_SUPPORT_2,         //!< double support below 2/4 time interval
      DOUBLE_SUPPORT_3,         //!< double support below 3/4 time interval
      DOUBLE_SUPPORT_4,         //!< double support below 4/4 time interval
      SINGLE_SUPPORT_1,         //!< single support below 1/4 time interval
      SINGLE_SUPPORT_2,         //!< single support below 2/4 time interval
      SINGLE_SUPPORT_3,         //!< single support below 3/4 time interval
      SINGLE_SUPPORT_4,         //!< single support below 4/4 time interval
    };

    /**
     * @brief Step 
     * 
     */
    enum Step
    {
      IDLE_STEP = 0,           
      FIRST = 3000,             //!< fist footstep
      LEFT,                     //!< left footstep
      RIGHT,                    //!< right footstep
      FINAL,                    //!< final footstep
    };

    /**
     * @brief Events produces by the ow_state_machine
     * 
     */
    enum EventOut
    {
      NONE_OUT = 0,           
      WALKING_END = 4000,     //!< walking plan finished
      HOMEING_START,          //!< Homing pose spline started
      DOUBLE_SUPPORT_START,   //!< Double support phase started == WALKING_START, == NEXT_FOOTSTEP_START
      DOUBLE_SUPPORT_MID,     //!< Double support phase in the middle
      SINGLE_SUPPORT_START,   //!< Single support phase started
      SINGLE_SUPPORT_MID,     //!< Single support phase in the middle
    };

    /**
     * @brief Events consumed by the ow_state_machine
     * 
     */
    enum EventIn
    {
      NONE_IN = 0,
      EARLY_TOUCH_DOWN = 5000,  //!< Foot hit the ground earlyer than expected
      STABLE_FOOT_HOLD,         //!< Foot has ground contact
      REACHED_HOME,             //!< Homing pose reached
      NEW_PLAN,                 //!< New plan arived
      PLAN_FINISHED             //!< Plan is finished
    };

  protected:
    // current states
    State state_;                       //!< current state
    WalkingPhase walking_phase_;        //!< current walking phase
    Quater quater_;                     //!< current quater
    Step step_;                         //!< current step

    EventOut event_out_;                //!< statemachine output event
    EventIn event_in_;                  //!< statemachine input event

    // robot properites
    size_t n_step_;                     //!< current step
    FootId support_foot_;               //!< support foot according to plan
    FootId swing_foot_;                 //!< swing foot according to plan

    std::vector<bool> feet_in_contact_; //!< flag for feet in conntact
    bool is_standing_;                  //!< both feet on the ground
    bool has_ground_contact_;           //!< at least one foot of the ground

    ros::Duration elapsed_plan_;
    ros::Duration elapsed_step_;            //!< time since foot step start
    ros::Duration elapsed_double_support_;  //!< time since double support
    ros::Duration elapsed_single_support_;  //!< time since single support

  public:
    Flags() : 
      state_(IDLE),
      walking_phase_(IDLE_PHASE),
      quater_(IDLE_QUATER),
      step_(IDLE_STEP),
      event_out_(NONE_OUT),
      event_in_(NONE_IN),
      n_step_(0),
      support_foot_(FootId::RIGHT),
      swing_foot_(FootId::LEFT),
      is_standing_(false),
      has_ground_contact_(false),
      elapsed_plan_(0),
      elapsed_step_(0),
      elapsed_double_support_(0),
      elapsed_single_support_(0)
    {
      feet_in_contact_.resize(2, false);
    }

    /*!
   * \brief Copy constructor.
   */
    Flags(const Flags& other) : 
      state_(other.state_),
      walking_phase_(other.walking_phase_),
      quater_(other.quater_),
      step_(other.step_),
      event_out_(other.event_out_),
      event_in_(other.event_in_),
      n_step_(other.n_step_),
      support_foot_(other.support_foot_),
      swing_foot_(other.swing_foot_),
      is_standing_(other.is_standing_),
      has_ground_contact_(other.has_ground_contact_),
      elapsed_step_(other.elapsed_step_),
      elapsed_double_support_(other.elapsed_double_support_),
      elapsed_single_support_(other.elapsed_single_support_),
      feet_in_contact_(other.feet_in_contact_)
    {
    }

    Flags& operator=(const Flags& other)
    {
      state_ = other.state_;
      walking_phase_ = other.walking_phase_;
      quater_ = other.quater_;
      step_ = other.step_;
      event_out_ = other.event_out_;
      event_in_ = other.event_in_;
      n_step_ = other.n_step_;
      support_foot_ = other.support_foot_;
      swing_foot_ = other.swing_foot_;
      is_standing_ = other.is_standing_;
      has_ground_contact_ = other.has_ground_contact_;
      elapsed_step_ = other.elapsed_step_;
      elapsed_double_support_ = other.elapsed_double_support_;
      elapsed_single_support_ = other.elapsed_single_support_;
      feet_in_contact_ = other.feet_in_contact_;
      return *this;
    }

    State& state()
    {
      return state_;
    }

    const State& state() const
    {
      return state_;
    }

    WalkingPhase& walkingPhase()
    {
      return walking_phase_;
    }

    const WalkingPhase& walkingPhase() const
    {
      return walking_phase_;
    }

    Quater& quater()
    {
      return quater_;
    }

    Step& step()
    {
      return step_;
    }
    
    const Step& step() const
    {
      return step_;
    }

    const Quater& quater() const
    {
      return quater_;
    }

    EventOut& eventOut()
    {
      return event_out_;
    }

    const EventOut& eventOut() const
    {
      return event_out_;
    }

    EventIn& eventIn()
    {
      return event_in_;
    }

    const EventIn& eventIn() const
    {
      return event_in_;
    }

    size_t& nStep()
    {
      return n_step_;
    }

    const size_t& nStep() const
    {
      return n_step_;
    }

    FootId& swingFoot()
    {
      return swing_foot_;
    }

    const FootId& swingFoot() const
    {
      return swing_foot_;
    }
    
    FootId& supportFoot()
    {
      return support_foot_;
    }

    const FootId& supportFoot() const
    {
      return support_foot_;
    }

    std::vector<bool>& feetInContact()
    {
      return feet_in_contact_;
    }

    const std::vector<bool>& feetInContact() const
    {
      return feet_in_contact_;
    }

    bool& isStanding()
    {
      return is_standing_;
    }

    const bool& isStanding() const
    {
      return is_standing_;
    }
    
    bool& hasGroundContact()
    {
      return has_ground_contact_;
    }

    const bool& hasGroundContact() const
    {
      return has_ground_contact_;
    }

    ros::Duration& elapsedPlan()
    {
      return elapsed_plan_;
    }

    const ros::Duration& elapsedPlan() const
    {
      return elapsed_plan_;
    }

    ros::Duration& elapsedStep()
    {
      return elapsed_step_;
    }

    const ros::Duration& elapsedStep() const
    {
      return elapsed_step_;
    }

    ros::Duration& elapsedDoubleSupport()
    {
      return elapsed_double_support_;
    }

    const ros::Duration& elapsedDoubleSupport() const
    {
      return elapsed_double_support_;
    }

    ros::Duration& elapsedSingleSupport()
    {
      return elapsed_single_support_;
    }

    const ros::Duration& elapsedSingleSupport() const
    {
      return elapsed_single_support_;
    }

    std::string toString() const
    {
      std::stringstream out;
      out << "state  :                  " << enumToString(state_) << "\n";
      out << "walking_phase  :          " << enumToString(walking_phase_) << "\n";
      out << "quater  :                 " << enumToString(quater_) << "\n";
      out << "step   :                  " << enumToString(step_) << "\n";
      out << "event_out  :              " << enumToString(event_out_) << "\n";
      out << "event_in :                " << enumToString(event_in_) << "\n";
      out << "n_step  :                 " << n_step_ << "\n";
      out << "support_foot  :           " << support_foot_.toString() << "\n";
      out << "swing_foot    :           " << swing_foot_.toString() << "\n";             
      out << "is_standing  :            " << is_standing_ << "\n";
      out << "has_ground_contact  :     " << has_ground_contact_ << "\n";
      out << "elapsed_plan:             " << elapsed_plan_ << "\n";
      out << "elapsed_step:             " << elapsed_step_ << "\n";
      out << "elapsed_double_support:   " << elapsed_double_support_ << "\n";
      out << "elapsed_single_support:   " << elapsed_single_support_ << "\n";
      out << "feet_in_contact[LEFT]:    " << feet_in_contact_[FootId::LEFT] << "\n";
      out << "feet_in_contact[RIGHT]:   " << feet_in_contact_[FootId::RIGHT] << "\n";
      return out.str();
    }

    /*!
    * \brief Assignment of ow_msgs::Flags.
    */
    Flags& operator=(const ow_msgs::Flags& msg)
    {
      state_ = static_cast<State>(msg.state.data);
      walking_phase_ = static_cast<WalkingPhase>(msg.walking_phase.data);
      quater_ = static_cast<Quater>(msg.quater.data);
      event_out_ = static_cast<EventOut>(msg.event_out.data);
      event_in_ = static_cast<EventIn>(msg.event_in.data);
      step_ = static_cast<Step>(msg.step.data);
      n_step_ = msg.n_step.data;
      support_foot_ = FootId(static_cast<FootId::Value>(msg.support_foot.data));
      swing_foot_ = FootId(static_cast<FootId::Value>(msg.swing_foot.data));
      is_standing_ = msg.is_standing.data;
      has_ground_contact_ = msg.has_ground_contact.data;
      elapsed_plan_ = msg.elapsed_plan.data;
      elapsed_step_ = msg.elapsed_step.data;
      elapsed_double_support_ = msg.elapsed_double_support.data;
      elapsed_single_support_ = msg.elapsed_single_support.data;
      feet_in_contact_.resize(2);
      feet_in_contact_[FootId::LEFT] = static_cast<bool>(msg.feet_in_contact_L.data);
      feet_in_contact_[FootId::RIGHT] = static_cast<bool>(msg.feet_in_contact_R.data);
      return *this;
    }

    /*!
    * \brief Conversion to ow_msgs::Flags.
    */
    operator ow_msgs::Flags() const
    {
      ow_msgs::Flags msg;
      msg.state.data = state_;
      msg.walking_phase.data = walking_phase_;
      msg.quater.data = quater_;
      msg.event_out.data = event_out_;
      msg.event_in.data = event_in_;
      msg.step.data = step_;
      msg.n_step.data = n_step_;
      msg.support_foot.data = support_foot_;
      msg.swing_foot.data = swing_foot_;
      msg.is_standing.data = is_standing_;
      msg.has_ground_contact.data = has_ground_contact_;
      msg.elapsed_plan.data = elapsed_plan_;
      msg.elapsed_step.data = elapsed_step_;
      msg.elapsed_double_support.data = elapsed_double_support_;
      msg.elapsed_single_support.data = elapsed_single_support_;
      msg.feet_in_contact_L.data = feet_in_contact_[FootId::LEFT];
      msg.feet_in_contact_R.data = feet_in_contact_[FootId::RIGHT];
      enumToMsg(msg, state_);
      enumToMsg(msg, walking_phase_);
      enumToMsg(msg, quater_);
      enumToMsg(msg, step_);
      enumToMsg(msg, event_out_);
      enumToMsg(msg, event_in_);
      return msg;
    }  

    /*!
    * \brief Conversion to ow_msgs::Flags.
    */
    ow_msgs::Flags toFlagsMsg() const
    {
      return static_cast<ow_msgs::Flags>(*this);
    }

    static std::string enumToString(State state)
    {
      switch (state)
      {
      case IDLE:
        return "IDLE";
      case HOMEING:
        return "HOMEING";
      case STANCE:
        return "STANCE";
      case WALKING:
        return "WALKING";
      }
      return "UNKOWN";
    }

    static void enumToMsg(ow_msgs::Flags& msg, State state)
    {
      switch (state)
      {
      case IDLE:
        msg.state_idle.data = 1;
        break;
      case HOMEING:
        msg.homing.data = 1;
        break;
      case STANCE:
        msg.ready.data = 1;
        break;
      case WALKING:
        msg.walking.data = 1;
        break;
      }
    }

    static std::string enumToString(WalkingPhase state)
    {
      switch (state)
      {
      case IDLE_PHASE:
        return "IDLE_PHASE";
      case DOUBLE_SUPPORT:
        return "DOUBLE_SUPPORT";
      case SINGLE_SUPPORT:
        return "SINGLE_SUPPORT";
      }
      return "UNKOWN";
    }

    static void enumToMsg(ow_msgs::Flags& msg, WalkingPhase state)
    {
      switch (state)
      {
      case IDLE_PHASE:
        msg.phase_idle.data = 1;
        break;
      case DOUBLE_SUPPORT:
        msg.double_support.data = 1;
        break;
      case SINGLE_SUPPORT:
        msg.single_support.data = 1;
        break;
      }
    }

    static std::string enumToString(Step step)
    {
      switch (step)
      {
      case IDLE_STEP:
        return "IDLE_STEP";
      case LEFT:
        return "LEFT";
      case RIGHT:
        return "RIGHT";
      case FIRST:
        return "FIRST";
      case FINAL:
        return "FINAL";
      }
      return "UNKOWN";
    }

    static void enumToMsg(ow_msgs::Flags& msg, Step step)
    {
      switch (step)
      {
      case IDLE_STEP:
        msg.idle_step.data = 1;
        break;
      case LEFT:
        msg.left.data = 1;
        break;
      case RIGHT:
        msg.right.data = 1;
        break;
      case FIRST:
        msg.first.data = 1;
        break;
      case FINAL:
        msg.final.data = 1;
        break;
      }
    }

    static std::string enumToString(Quater state)
    {
      switch (state)
      {
      case IDLE_QUATER:
        return "IDLE_QUATER";
      case DOUBLE_SUPPORT_1:
        return "DOUBLE_SUPPORT_1";
      case DOUBLE_SUPPORT_2:
        return "DOUBLE_SUPPORT_2";
      case DOUBLE_SUPPORT_3:
        return "DOUBLE_SUPPORT_3";
      case DOUBLE_SUPPORT_4:
        return "DOUBLE_SUPPORT_4";
      case SINGLE_SUPPORT_1:
        return "SINGLE_SUPPORT_1";
      case SINGLE_SUPPORT_2:
        return "SINGLE_SUPPORT_2";
      case SINGLE_SUPPORT_3:
        return "SINGLE_SUPPORT_3";
      case SINGLE_SUPPORT_4:
        return "SINGLE_SUPPORT_4";
      }
      return "UNKOWN";
    }

    static void enumToMsg(ow_msgs::Flags& msg, Quater state)
    {
      switch (state)
      {
      case IDLE_QUATER:
        msg.quater_idle.data = 1;
        break;
      case DOUBLE_SUPPORT_1:
        msg.double_support_1.data = 1;
        break;
      case DOUBLE_SUPPORT_2:
        msg.double_support_2.data = 1;
        break;
      case DOUBLE_SUPPORT_3:
        msg.double_support_3.data = 1;
        break;
      case DOUBLE_SUPPORT_4:
        msg.double_support_4.data = 1;
        break;
      case SINGLE_SUPPORT_1:
        msg.single_support_1.data = 1;
        break;
      case SINGLE_SUPPORT_2:
        msg.single_support_2.data = 1;
        break;
      case SINGLE_SUPPORT_3:
        msg.single_support_3.data = 1;
        break;
      case SINGLE_SUPPORT_4:
        msg.single_support_4.data = 1;
        break;
      }
    }

    static std::string enumToString(EventIn state)
    {
      switch (state)
      {
      case NONE_IN:
        return "NONE_IN";
      case EARLY_TOUCH_DOWN:
        return "EARLY_TOUCH_DOWN";
      case STABLE_FOOT_HOLD:
        return "STABLE_FOOT_HOLD";
      case REACHED_HOME:
        return "REACHED_HOME";
      case NEW_PLAN:
        return "NEW_PLAN";
      case PLAN_FINISHED:
        return "PLAN_FINISHED";
      }
      return "UNKOWN";
    }

    static void enumToMsg(ow_msgs::Flags& msg, EventIn state)
    {
      switch (state)
      {
      case NONE_IN:
        msg.event_in_none.data = 1;
        break;
      case EARLY_TOUCH_DOWN:
        msg.early_touch_down.data = 1;
        break;
      case STABLE_FOOT_HOLD:
        msg.stable_foot_hold.data = 1;
        break;
      case REACHED_HOME:
        msg.reached_home.data = 1;
        break;
      case NEW_PLAN:
        msg.new_plan.data = 1;
        break;
      case PLAN_FINISHED:
        msg.plan_finished.data = 1;
        break;
      }
    }

    static std::string enumToString(EventOut state)
    {
      switch (state)
      {
      case NONE_OUT:
        return "NONE_OUT";
      case HOMEING_START:
        return "HOMEING_START";
      case WALKING_END:
        return "WALKING_END";
      case DOUBLE_SUPPORT_START:
        return "DOUBLE_SUPPORT_START";
      case DOUBLE_SUPPORT_MID:
        return "DOUBLE_SUPPORT_MID";
      case SINGLE_SUPPORT_START:
        return "SINGLE_SUPPORT_START";
      case SINGLE_SUPPORT_MID:
        return "SINGLE_SUPPORT_MID";
      }
      return "UNKOWN";
    }

    static void enumToMsg(ow_msgs::Flags& msg, EventOut state)
    {
      switch (state)
      {
      case NONE_OUT:
        msg.event_out_none.data = 1;
        break;
      case WALKING_END:
        msg.walking_end.data = 1;
        break;
      case DOUBLE_SUPPORT_START:
        msg.double_support_start.data = 1;
        break;
      case DOUBLE_SUPPORT_MID:
        msg.double_support_mid.data = 1;
        break;
      case SINGLE_SUPPORT_START:
        msg.single_support_start.data = 1;
        break;
      case SINGLE_SUPPORT_MID:
        msg.single_support_mid.data = 1;
        break;
      }
    }

  };

} // namespace ow_core

#endif // OPEN_WALKER_FLAGS_H
