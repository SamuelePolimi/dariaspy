import time

from dariaspy.positions import Home_Position
from dariaspy.darias_interface import Darias
from dariaspy.observers import OptitrackObserver, EndEffectorObserver
from dariaspy.trajectory import GoToTrajectory


def obs_renaming(**refs):
    return {("R_" + ("".join(k.split("_")[-2:]))):v for k, v in refs.items()}


#we want to have only the position of the object, not its orientation
def mixed_observation(object_obs, hand_obs):
    obj_refs = ['R_TX', 'R_TY', 'R_TZ']
    hand_refs = ['R_RX', 'R_RY', 'R_RZ', 'R_RW']
    ret = {}
    for ref in obj_refs:
        ret[ref] = object_obs[ref]
    for ref in hand_refs:
        ret[ref] = hand_obs[ref]
    return ret


if __name__ == "__main__":
    darias = Darias()

    darias.go_to(GoToTrajectory(**Home_Position), "RIGHT_ARM")
    oo = OptitrackObserver("human")
    eo = EndEffectorObserver(darias)

    print("Publish a object called 'human'. the robot will move toward this object.\n Bring with you the console and be ready for press the BREAK. Put the object in a reachable position for the arm.")
    raw_input("If you are ready press ENTER. The demo will start in 10 seconds by that.")

    time.sleep(10)
    observation = obs_renaming(**oo(*oo.get_possible_refs()))
    observation = mixed_observation(observation, eo(*eo.get_possible_refs()))
    trajectory = GoToTrajectory(10., **observation)
    darias.go_to(trajectory, "ENDEFF_RIGHT_ARM")