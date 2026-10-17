#include "ProtocolMessages.h"

const char *runStateName(RunStateV2 state)
{
  switch (state)
  {
    case RUN_IDLE: return "IDLE";
    case RUN_PLAN_READY: return "PLAN_READY";
    case RUN_RUNNING: return "RUNNING";
    case RUN_PAUSE_PENDING: return "PAUSE_PENDING";
    case RUN_PAUSED: return "PAUSED";
    case RUN_STOPPED: return "STOPPED";
    case RUN_COMPLETED: return "COMPLETED";
    case RUN_FAULT: return "FAULT";
    default: return "FAULT";
  }
}

const char *runPhaseName(RunPhaseV2 phase)
{
  switch (phase)
  {
    case PHASE_NONE: return "NONE";
    case PHASE_SELECT_TARGET: return "SELECT_TARGET";
    case PHASE_MOVING: return "MOVING";
    case PHASE_SETTLING: return "SETTLING";
    case PHASE_TRIGGER_ACTIVE: return "TRIGGER_ACTIVE";
    case PHASE_EXPOSURE_WAIT: return "EXPOSURE_WAIT";
    case PHASE_COMMIT_IMAGE: return "COMMIT_IMAGE";
    default: return "NONE";
  }
}
