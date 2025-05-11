class Action:
    move_arm: int = 0
    move_hand: int = 0

    def __init__(self, arm, hand):
        self.move_arm = arm
        self.move_hand = hand
