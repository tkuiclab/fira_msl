#include "fsm/fsm_team_strategy.h"

FSM(TeamStrategy){
FSM_STATES
{
    KickOff,
    FreeKick,
    Offensive,
    Defensive,
    Goalie,
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
            FSM_ON_EVENT("/GOALIE", FSM_NEXT(Goalie))
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
            FSM_ON_EVENT("/GOALIE", FSM_NEXT(Goalie))
        }
    }
    FSM_STATE(Offensive)
    {
        FSM_CALL_TASK(Offensive)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/STAND_BY", FSM_NEXT(StandBy))
        }
    }
    FSM_STATE(Defensive)
    {
        FSM_CALL_TASK(Defensive)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/STAND_BY", FSM_NEXT(StandBy))
        }
    }
    FSM_STATE(Goalie)
    {
        FSM_CALL_TASK(Goalie)

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
