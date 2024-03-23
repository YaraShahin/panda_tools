def _approach(self, pose):
    """
    Move to a pose with a hover-distance above the requested pose and
    then move to the pose incrementally while monitoring the z force
    """
    print('approaching...')
    approach = copy.deepcopy(pose)
    approach.position.z = approach.position.z + self._hover_distance + self.robot.eps.z + self.robot.hand_EE_offset
    self.robot.move_to(approach)

    while approach.position.z >= pose.position.z:
        approach.position.z = approach.position.z - self.step_size
        self.robot.move_to(approach)