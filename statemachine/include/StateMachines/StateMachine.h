#ifndef SUBSTATEMACHINE_H
#define SUBSTATEMACHINE_H

/**
  *@brief Macros to help with the usage of StateMachines
  */
#define SM_DURING   if (!stateChangeFlag_)\
                    {\
                    incTimer();\
                    switch (currentState_)\
                    {
#define SM_EXIT     default:\
                    break;\
                    }\
                    }\
                    if (stateChangeFlag_)\
                    {\
                    switch (lastState_)\
                    {
#define SM_ENTRY    default:\
                    break;\
                    }\
                    switch (currentState_)\
                    {
#define SM_END      default:\
                    break;\
                    }\
                    stateChangeFlag_ = false;\
                    }\
                    lastState_ = currentState_;
#define SM_AFTER(time) if (wait(time))
#define SM_DO_UNTIL(time) if (doUntil(time))
#define SM_DO_IN_INTERVAL(time) if (doInInterval(time))

/**
  *@brief Macros to help with the usage of subStatemachines
  */
#define SUBSM_DURING(a); if (!a.stateChangeFlag_)\
                        {\
                        a.incTimer();\
                        switch (a.currentState_)\
                        {
#define SUBSM_EXIT(a);   default:\
                        break;\
                        }\
                        }\
                        if (a.stateChangeFlag_)\
                        {\
                        switch (a.lastState_)\
                        {
#define SUBSM_ENTRY(a);  default:\
                        break;\
                        }\
                        switch (a.currentState_)\
                        {
#define SUBSM_END(a);    default:\
                        break;\
                        }\
                        a.stateChangeFlag_ = false;\
                        }\
                        a.lastState_ = a.currentState_;
#define SUBSM_AFTER(a, time) if (a.wait(time))
#define SUBSM_DO_UNTIL(a, time) if (a.doUntil(time))
#define SUBSM_DO_IN_INTERVAL(a, time) if (a.doInInterval(time))

template <class T>
class StateMachine
{

public:

	/**
	  *@brief Constructor for StateMachine
	  */
    StateMachine(T initState, double frequency)
        :
          currentState_(initState),
          lastState_(initState),
          stateChangeFlag_(true),
          frequency_(frequency)
    {
        intervalInSec_ = 1/frequency;
    }

	/**
	  *@brief get the current state of the StateMachine
	  */
    T getState(){return currentState_;}

	/**
	  *@brief change the state of the StateMachine setting the state change flag and resetting the timer
	  */
    void changeState(T newState){lastState_ = currentState_; currentState_ = newState; stateChangeFlag_ = true; timer_ = 0;intervaltimer_ = 0;}

	/**
      *@brief returns true when state waitet for time_in_seconds
	  */
    bool wait(double time_in_seconds){return (time_in_seconds <= timer_);}

    /**
      *@brief returns true until time in seconds are over
      */
    bool doUntil(double time_in_seconds){return (time_in_seconds > timer_);}

    /**
      *@brief returns true until time in seconds are over
      */
    bool doInInterval(double time_in_seconds){if (time_in_seconds < intervaltimer_) {intervaltimer_ = 0; return true; } else return false;}

    /**
      *@brief returns true until time in seconds are over
      */
    bool doInGlobalInterval(double time_in_seconds){if (time_in_seconds < globalIntervaltimer_) {globalIntervaltimer_ = 0; return true; } else return false;}

	/**
	  *@brief increment timer
	  */
    void incTimer(){timer_ += intervalInSec_; intervaltimer_ += intervalInSec_; globalIntervaltimer_ += intervalInSec_;}

    T currentState_; /**< current state of the StateMachine*/

    T lastState_; /**< last state of the StateMachine*/

	bool stateChangeFlag_; /**< State change flag being set to true for one period*/

protected:

    double intervalInSec_; /**< Statemachine interval in seconds */

    double frequency_; /**< Statemachine frequency */

    double timer_ = 0; /**< timer counting intervals*/

    double intervaltimer_ = 0; /**< timer counting intervals*/

    double globalIntervaltimer_ = 0; /**< global timer counting intervals*/
};

#endif // SUBSTATEMACHINE_H

