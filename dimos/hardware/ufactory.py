from dimos.hardware.end_effector import EndEffector

class UFactoryEndEffector(EndEffector):
    def __init__(self, model=None, **kwargs):
        super().__init__(**kwargs)
        self.model = model

    def get_model(self):
        return self.model

class UFactory7DOFArm:
    def __init__(self, arm_length=None):
        self.arm_length = arm_length

    def get_arm_length(self):
        return self.arm_length
