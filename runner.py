#!/usr/bin/env python3
import numpy as np
from mpscenes.goals.goal_composition import GoalComposition
from fabrics.planner.parameterized_planner import ParameterizedFabricPlanner
from ros.ros_converter_node import ActionConverterNode

class fabrics_runner():
    def __init__(self):
        goal_dict = {
                "subgoal0": {
                    "weight": 1.0,
                    "is_primary_goal": True,
                    "indices": [0, 1],
                    "parent_link" : 0,
                    "child_link" : 1,
                    "desired_position": [2.0, 0.0],
                    "epsilon" : 0.1,
                    "type": "staticSubGoal"
                }
        }
        self._goal = GoalComposition(name="goal", content_dict=goal_dict)
        self._collision_links = [1]
        # 195 for range 10, 93 for range 21, 49 for range 40
        self._number_lidar_rays = 49
        self.startRosConverterNode()

    def initialize_runtime_arguments(self):
        self._runtime_arguments = {}
        self._runtime_arguments['weight_goal_0'] = np.array([1.0])
        self._runtime_arguments['base_inertia'] = np.array([0.4])
        for j in range(self._number_lidar_rays):
            for i in self._collision_links:
                self._runtime_arguments[f'obst_geo_exp_obst_{j}_{i}_leaf'] = np.array([1.0])
                self._runtime_arguments[f'obst_geo_lam_obst_{j}_{i}_leaf'] = np.array([2.0])
                self._runtime_arguments[f'radius_body_{i}'] = np.array([0.4])
    
    def startRosConverterNode(self):
        dt = 0.1
        rate_int = int(1/dt)
        self._rosConverter = ActionConverterNode(dt, rate_int)
   
    def applyAction(self, action):
        ob = self._rosConverter.publishAction(action)
        return ob 

    def reset(self):
        ob = self._rosConverter.ob()
        return ob

    def set_planner(self):
        degrees_of_freedom = 2
        robot_type = "pointRobot"
        collision_geometry = "-10.0 / (x**1) * xdot**2"
        collision_finsler = "1.0 / (x**5) * (1 - ca.heaviside(xdot)) * xdot**2"
        planner = ParameterizedFabricPlanner(
                degrees_of_freedom,
                robot_type,
                collision_geometry=collision_geometry,
                collision_finsler=collision_finsler
        )
        collision_links = [1]
        planner.set_components(
            collision_links=collision_links,
            goal=self._goal,
            number_obstacles=self._number_lidar_rays,
        )
        planner.concretize()
        self.initialize_runtime_arguments()
        print(f"Planner is concretized: {planner}")
        self._planner = planner

    def adapt_runtime_arguments(self, ob):
        self._runtime_arguments['q'] = ob[0]['x'][0:2]
        self._runtime_arguments['qdot'] = ob[0]['xdot'][0:2]
        self._runtime_arguments['x_goal_0'] = np.array(self._goal.primary_goal().position())
        #self._runtime_arguments['x_goal_0'] = self._goal_pos
        #breakpoint()
        for j in range(self._number_lidar_rays):
            #dist = np.linalg.norm(np.array([0.0, 0.0]) - ob[0]['obs'][j][0:2])
            #print(f"{j}: {dist}")
            self._runtime_arguments[f'radius_obst_{j}'] = np.array([0.15])
            self._runtime_arguments[f'x_obst_{j}'] = ob[0]['obs'][j][0:2]

    def run(self, n_steps=100000):
        self.set_planner()
        ob = self.reset()
       
        #goal_abs = np.array(self._goal.primary_goal().position())
        #current_odom = ob[0]['x']
        #self._goal_pos = goal_abs + current_odom

        for n_step in range(n_steps):
            #print(f"{ob[0]['x']}")
            self.adapt_runtime_arguments(ob)
            action = self._planner.compute_action(**self._runtime_arguments)
            ob = self.applyAction(action)
        return {}

if __name__ == "__main__":
    fabrics_run = fabrics_runner()
    fabrics_run.run()

