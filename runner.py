#!/usr/bin/env python3

import numpy as np
from mpscenes.goals.goal_composition import GoalComposition
from fabrics.planner.parameterized_planner import ParameterizedFabricPlanner
from ros.ros_converter_node import ActionConverterNode


class fabrics_runner():
    def __init__(self):
        pos0 = np.array([0.0, 0.0, 0.0])
        vel0 = np.array([0.0, 0.0, 0.0])
        
        # Definition of the goal.
        goal_dict = {
                "subgoal0": {
                    "weight": 0.5,
                    "is_primary_goal": True,
                    "indices": [0, 1],
                    "parent_link" : 0,
                    "child_link" : 1,
                    "desired_position": [3.5, 0.5],
                    "epsilon" : 0.1,
                    "type": "staticSubGoal"
                }
        }
        self._goal = GoalComposition(name="goal", content_dict=goal_dict)
        self._number_lidar_rays = 1
        self.startRosConverterNode()
    
    def startRosConverterNode(self):
        dt = 0.1
        rate_int = int(1/dt)
        self._rosConverter = ActionConverterNode(dt, rate_int)
   
    def applyAction(self, action):
        ob = self._rosConverter.publishAction(action)
        #self._rosConverter.setGoal(self._goal)
        return ob 

    def reset(self):
        ob = self._rosConverter.ob()
        #self._rosConverter.setGoal(self._goal)
        return ob

    def set_planner(self):
        """
        Initializes the fabric planner for the point robot.

        This function defines the forward kinematics for collision avoidance,
        and goal reaching. These components are fed into the fabrics planner.

        In the top section of this function, an example for optional reconfiguration
        can be found. Commented by default.
        """
        degrees_of_freedom = 2
        robot_type = "pointRobot"
        #collision_geometry = "-2.0 / (x ** 1) * xdot ** 2"
        #collision_finsler = "1.0/(x**2) * (1 - ca.heaviside(xdot))* xdot**2"
        planner = ParameterizedFabricPlanner(
                degrees_of_freedom,
                robot_type,
                #collision_geometry=collision_geometry,
                #collision_finsler=collision_finsler
        )
        collision_links = [1]
        planner.set_components(
            collision_links=collision_links,
            goal=self._goal,
            number_obstacles=self._number_lidar_rays,
        )
        planner.concretize(mode='vel', time_step=0.01)
        print(f"Planner is concretized: {planner}")
        self._planner = planner

    def run(self, n_steps=10000):
        """
        Set the goal, the planner and run.
        
        Params
        ----------
        n_steps
            Total number of simulation steps.
        """
        self.set_planner()

        action = np.array([0.0, 0.0, 0.0])
        ob = self.reset()
        #ob = self.applyAction(np.zeros((3, 0)))

        for _ in range(n_steps):
            ob_robot = ob
            action[0:2] = self._planner.compute_action(
                q=ob_robot["joint_state"]["position"][0:2],
                qdot=ob_robot["joint_state"]["velocity"][0:2],
                x_goal_0=ob_robot['FullSensor']['goals'][0][0][0:2],
                weight_goal_0=goal.sub_goals()[0].weight(),
                x_obst_0=ob_robot['FullSensor']['obstacles'][0][0][0:2],
                radius_obst_0=ob_robot['FullSensor']['obstacles'][0][1],
                radius_body_1=np.array([0.2])
            )
            ob = self.applyAction(action)
        return {}


if __name__ == "__main__":
    fabrics_run = fabrics_runner()
    fabrics_run.run()

