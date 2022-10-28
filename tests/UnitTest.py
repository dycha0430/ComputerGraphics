from MotionViewer import *
import pytest


###########################################################
# Unit test
def test_pytest():
    assert 3 == 3


# Test skeleton's joint count is correct
def test_parsed_joint_num_is_correct():
    motion = BvhMotion("unit_test.bvh")
    assert len(motion.skeleton.joints) == 57


# Test root joint is correct
def test_get_root():
    motion = BvhMotion("unit_test.bvh")
    root = motion.get_root()
    assert len(root.channels) == 6
    assert root.offset == [0, 0, 0]
    assert root.isEndEffector == False
    assert len(root.children) == 3


# Test parsed joint tree is made corretly
def test_joint_tree_is_correct():
    motion = BvhMotion("unit_test.bvh")
    root = motion.get_root()
    first_child = root.children[0]
    assert first_child.offset == [0, 20.6881, -0.73152]
    assert len(first_child.children) == 1
    assert first_child.children[0].children[0].children[0].children[0].children[0].isEndEffector == True


# Test parsed posture's data count is equal to skeleton's total channel count
def test_parsed_posture_data_count_is_correct():
    motion = BvhMotion("unit_test.bvh")
    assert len(motion.postures[0].values) == 132


# Test parsed posture data is correct
def test_parsed_posture_data_is_correct():
    motion = BvhMotion("unit_test.bvh")
    assert motion.postures[0].get_value(0) == 53.6842
