#include "fsm/fsm_goal_keeper.h"

FSM(GoalKeeper)
{
	FSM_STATES
	{
        FindBall,
        GotBall,
	}
	FSM_START(FindBall);
	FSM_BGN
	{
		FSM_STATE(FindBall)
		{
            FSM_CALL_TASK(Goalie)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/GOT_BALL", FSM_NEXT(GotBall));
			}
		}
		FSM_STATE(GotBall)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/LOST_BALL", FSM_NEXT(FindBall));
			}
		}
	}
	FSM_END
}

