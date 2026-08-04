#RACECAR Quest Log Trial 2B: Wall Follower

Our current iteration of the autonomous wall follower uses an algorithm built off of a follow-the-gap wall follower. *

We started with bang-bang control and a simple proportional controller that corrected cross-track error. When it had issues with consistency, we added a low-pass filter but realized this method was simply not sufficient for higher speeds. So we moved on to a follow-the-gap algorithm and iterated by adding weights to distances until we had our current version.
