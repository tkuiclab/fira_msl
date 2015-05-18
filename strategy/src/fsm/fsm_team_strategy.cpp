#include "fsm/fsm_team_strategy.h"

FSM(GoalKeeper)
{
	FSM_STATES
	{
        FindBall,
        GotBall,
	}
	FSM_START(FindBall)
	FSM_BGN
	{
		FSM_STATE(FindBall)
		{
            FSM_CALL_TASK(Goalie)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/GOT_BALL", FSM_NEXT(GotBall))
			}
		}
		FSM_STATE(GotBall)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/LOST_BALL", FSM_NEXT(FindBall))
			}
		}
	}
	FSM_END
}

FSM(Offensive)
{
	FSM_STATES
	{
        FindBall,
        GotBall,
	}
	FSM_START(FindBall)
	FSM_BGN
	{
		FSM_STATE(FindBall)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/GOT_BALL", FSM_NEXT(GotBall))
			}
		}
		FSM_STATE(GotBall)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/LOST_BALL", FSM_NEXT(FindBall))
			}
		}
	}
	FSM_END
}

FSM(Defensive)
{
	FSM_STATES
	{
        FindBall,
        GotBall,
	}
	FSM_START(FindBall)
	FSM_BGN
	{
		FSM_STATE(FindBall)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/GOT_BALL", FSM_NEXT(GotBall))
			}
		}
		FSM_STATE(GotBall)
		{
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/LOST_BALL", FSM_NEXT(FindBall))
			}
		}
	}
	FSM_END
}

FSM(Roles){
FSM_STATES
{
    NoRole,
    Offensive,
    Defensive,
    GoalKeeper,
}
FSM_START(NoRole);
FSM_BGN
{
    FSM_STATE(Offensive)
    {
        FSM_CALL_FSM(Offensive)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/BALL_ON_SELF_HALF", FSM_NEXT(Defensive))
            FSM_ON_EVENT("/DEFENSIVE", FSM_NEXT(Defensive))
            FSM_ON_EVENT("/OFFENSIVE", FSM_NEXT(Offensive))
        }
    }
    FSM_STATE(Defensive)
    {
        FSM_CALL_FSM(Defensive)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/OFFENSIVE", FSM_NEXT(Offensive))
        }
    }
    FSM_STATE(GoalKeeper)
    {
        FSM_CALL_FSM(GoalKeeper)
        FSM_TRANSITIONS
        {
        }
    }
    FSM_STATE(NoRole)
    {
        FSM_CALL_TASK(NoRole)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/DEFENSIVE", FSM_NEXT(Defensive))
            FSM_ON_EVENT("/OFFENSIVE", FSM_NEXT(Offensive))
            FSM_ON_EVENT("/GOAL_KEEPER", FSM_NEXT(GoalKeeper))
        }
    }
}
FSM_END
}

FSM(GameState)
{
	FSM_STATES
	{
        StandBy,
        Start,
        FreeKick
	}
	FSM_START(StandBy);
	FSM_BGN
	{
		FSM_STATE(StandBy)
		{
			FSM_TRANSITIONS
			{
                FSM_ON_EVENT("/KICK_OFF", FSM_NEXT(Start))
                FSM_ON_EVENT("/FREE_KICK", FSM_NEXT(FreeKick))
			}
		}
		FSM_STATE(Start)
		{
            FSM_CALL_FSM(Roles)
			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/GOAL", FSM_NEXT(StandBy))
			}
		}
		FSM_STATE(FreeKick)
		{
			FSM_TRANSITIONS
			{
                FSM_ON_EVENT("/START", FSM_NEXT(Start))
			}
		}
	}
	FSM_END
}
