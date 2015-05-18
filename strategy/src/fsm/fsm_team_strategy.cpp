#include "fsm/fsm_team_strategy.h"

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

FSM(Offensive)
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

FSM(Defensive)
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

FSM(TeamStrategy){
FSM_STATES
{
    KickOff,
    FreeKick,
    Offensive,
    Defensive,
    GoalKeeper,
    StandBy,
}
FSM_START(StandBy);
FSM_BGN
{
    FSM_STATE(KickOff)
    {
        FSM_CALL_TASK(KickOff)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/STAND_BY", FSM_NEXT(StandBy))
            FSM_ON_EVENT("/OFFENSIVE", FSM_NEXT(Offensive))
            FSM_ON_EVENT("/DEFENSIVE", FSM_NEXT(Defensive))
            FSM_ON_EVENT("/GOAL_KEEPER", FSM_NEXT(GoalKeeper))
        }
    }
    FSM_STATE(FreeKick)
    {
        FSM_CALL_TASK(FreeKick)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/STAND_BY", FSM_NEXT(StandBy))
            FSM_ON_EVENT("/OFFENSIVE", FSM_NEXT(Offensive))
            FSM_ON_EVENT("/DEFENSIVE", FSM_NEXT(Defensive))
            FSM_ON_EVENT("/GOAL_KEEPER", FSM_NEXT(GoalKeeper))
        }
    }
    FSM_STATE(Offensive)
    {
        FSM_CALL_FSM(Offensive)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/STAND_BY", FSM_NEXT(StandBy))
        }
    }
    FSM_STATE(Defensive)
    {
        FSM_CALL_FSM(Defensive)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/STAND_BY", FSM_NEXT(StandBy))
        }
    }
    FSM_STATE(GoalKeeper)
    {
        FSM_CALL_FSM(GoalKeeper)

        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/STAND_BY", FSM_NEXT(StandBy))
        }
    }
    FSM_STATE(StandBy)
    {
        FSM_CALL_TASK(StandBy)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/KICK_OFF", FSM_NEXT(KickOff))
            FSM_ON_EVENT("/FREE_KICK", FSM_NEXT(FreeKick))
        }
    }
}
FSM_END
}
