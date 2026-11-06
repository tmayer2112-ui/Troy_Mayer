"""Module 7: deciding which feet are planted.

  1. Do WORKSHEET.md, Module 7 on paper first.
  2. Fill in the TODO below (about 12 lines).
  3. Run:  python check.py 7   then:  python run.py contacts   (about a minute)

The filter's biggest assumption is "a planted foot doesn't move". This module decides when a foot
counts as planted, the way the real project does (python/qeskf/pipeline.py, closed_loop.py):
force above 5 N AND the gait schedule says stance, ending a few ms before the scheduled liftoff.
Then you reproduce two real results: the vertical-drift bug (DEBUG_LOG #4) and the latency experiment (E4).
"""
import numpy as np


def delay(signal, n):
    """(Provided.) The signal as it arrives n samples late: the first row is repeated at the start."""
    if n <= 0:
        return signal
    return np.concatenate([np.repeat(signal[:1], n, 0), signal[:-n]])


def ahead(sched, n):
    """(Provided.) The schedule n samples in the future: row k is sched[k + n]. The end repeats."""
    if n <= 0:
        return sched
    return np.concatenate([sched[n:], np.repeat(sched[-1:], n, 0)])


def detect_contacts(force, sched, threshold=5.0, latency=0, advance=0, settle=0,
                    use_force=True, use_schedule=True):
    """(n, 4) bool: which feet the filter treats as planted at each sample.

    force     (n, 4) foot forces in N        sched  (n, 4) bool, the gait schedule's stance
    latency   samples the force signal arrives late
    advance   end contact this many samples before the schedule's liftoff
    settle    a foot must have been in contact for more than this many samples before it counts
    """
    # TODO (about 12 lines), worksheet 7.2:
    #   start from c = np.ones(sched.shape, bool)
    #   if use_force:    keep only samples where the (late) force is above threshold: delay(force, latency)
    #   if use_schedule: keep only samples where the schedule says stance now AND `advance` samples from now
    #   if settle:       count, per leg, how many samples in a row each foot has been in contact,
    #                    and only report contact once that count is above settle
    raise NotImplementedError