#
# Copyright (c) 2023 INRIA
# Author: Louis Montaut
#

import unittest
import pycolbench
import pinocchio as pin
import numpy as np
import pickle
import random

SAVE_PATH = "./build/python_test_save.bin"

def random_collision_problem() -> pycolbench.CollisionProblem:
    prob = pycolbench.CollisionProblem()
    prob.pair_id = random.randint(0, 1000)
    prob.id_shape1 = random.randint(0, 1000)
    prob.id_shape2 = random.randint(0, 1000)
    prob.id_pose = random.randint(0, 1000)
    prob.p1 = np.random.rand(3)
    prob.p2 = np.random.rand(3)
    prob.p1_early = np.random.rand(3)
    prob.p2_early = np.random.rand(3)
    prob.id_pose = random.randint(0, 1000)
    prob.M1 = pin.SE3.Random()
    prob.M2 = pin.SE3.Random()
    prob.unscaled_translation = np.random.rand(3)
    prob.unscaled_separation_vector = np.random.rand(3)
    prob.normalized_separation_vector = prob.unscaled_separation_vector / np.linalg.norm(prob.unscaled_separation_vector)
    prob.unscaled_dist = np.random.rand(1)[0]
    prob.unscaled_separation_vector_early_stop = np.random.rand(3)
    prob.normalized_separation_vector_early_stop = prob.unscaled_separation_vector_early_stop / np.linalg.norm(prob.unscaled_separation_vector_early_stop)
    prob.unscaled_dist_early_stop = np.random.rand(1)[0]
    return prob

class TestCollisionProblemPickling(unittest.TestCase):
    def check_same_collision_problem(self, obj1: pycolbench.CollisionProblem,
                                     obj2: pycolbench.CollisionProblem):
        self.assertTrue(obj1 == obj2)

    def pickling(self, obj1: pycolbench.CollisionProblem):
        # Pickle
        with open(SAVE_PATH, "wb") as f:
            pickle.dump(obj1, f)
        with open(SAVE_PATH, "rb") as f:
            obj2: pycolbench.CollisionProblem = pickle.load(f)

        self.check_same_collision_problem(obj1, obj2)

        # Serialization
        obj1.saveToBinary(SAVE_PATH)
        obj2 = pycolbench.CollisionProblem()
        obj2.loadFromBinary(SAVE_PATH)

        self.check_same_collision_problem(obj1, obj2)

    def pickling_vec(self, obj1: pycolbench.StdVec_CollisionProblem):
        # Pickle
        with open(SAVE_PATH, "wb") as f:
            pickle.dump(obj1, f)
        with open(SAVE_PATH, "rb") as f:
            obj2: pycolbench.StdVec_CollisionProblem = pickle.load(f)

        for i in range(len(obj1)):
            self.check_same_collision_problem(obj1[i], obj2[i])

        # Serialization
        obj1.saveToBinary(SAVE_PATH)
        obj2 = pycolbench.StdVec_CollisionProblem()
        obj2.loadFromBinary(SAVE_PATH)

        for i in range(len(obj1)):
            self.check_same_collision_problem(obj1[i], obj2[i])

    def pickling_vec_vec(self, obj1: pycolbench.StdVec_StdVec_CollisionProblem):
        # Pickle
        with open(SAVE_PATH, "wb") as f:
            pickle.dump(obj1, f)
        with open(SAVE_PATH, "rb") as f:
            obj2: pycolbench.StdVec_StdVec_CollisionProblem = pickle.load(f)

        for i in range(len(obj1)):
            for j in range(len(obj1[i])):
                self.check_same_collision_problem(obj1[i][j], obj2[i][j])

        # Serialization
        obj1.saveToBinary(SAVE_PATH)
        obj2 = pycolbench.StdVec_StdVec_CollisionProblem()
        obj2.loadFromBinary(SAVE_PATH)

        for i in range(len(obj1)):
            for j in range(len(obj1[i])):
                self.check_same_collision_problem(obj1[i][j], obj2[i][j])

    def test_pickle_collision_problem(self):
        prob = random_collision_problem()
        self.pickling(prob)

    def test_pickle_collision_problem_vector(self):
        probs = pycolbench.StdVec_CollisionProblem()
        for _ in range(3):
            prob = random_collision_problem()
            probs.append(prob)

        self.pickling_vec(probs)

    def test_pickle_collision_problem_vector_vector(self):
        trajs = pycolbench.StdVec_StdVec_CollisionProblem()
        for _ in range(3):
            probs = pycolbench.StdVec_CollisionProblem()
            for _ in range(4):
                prob = random_collision_problem()
                probs.append(prob)
            trajs.append(probs)

        self.pickling_vec_vec(trajs)


if __name__ == "__main__":
    unittest.main()
