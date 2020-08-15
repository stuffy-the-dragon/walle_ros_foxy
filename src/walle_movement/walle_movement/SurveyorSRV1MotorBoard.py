from gpiozero import LED, PhaseEnableMotor


class SurveyorSRV1MotorBoard:
    """ A class that controls the motor board on the Surveyor
    SRV-1 tracked robot
    """
    def __init__(self):
        # A factor that can be adjusted to fine tune motor speed
        self._left_speed = 1.0
        self._right_speed = 1.0

        # The actual motor objects
        self._left_motor = PhaseEnableMotor(26, 20)
        self._right_motor = PhaseEnableMotor(16, 21)

        # The laser objects
        self._left_laser = LED(13)
        self._right_laser = LED(12)

    def lasersOn(self):
        """ Turn on both lasers """
        self._left_laser.on()
        self._right_laser.on()

    def lasersOff(self):
        """ Turn off both lasers """
        self._left_laser.off()
        self._right_laser.off()

    @property
    def motorsSpeedFactors(self):
        """ Get the motor speed factors
        Output: {'left': float, 'right': float}
        """
        return {'left': self._left_speed, 'right': self._right_speed}

    @motorsSpeedFactors.setter
    def motorsSpeedFactors(self, left=1.0, right=1.0):
        """ Tune the motor speed factors
        Input: Keyword parameters => left = 1.0 and right = 1.0
        """
        try:
            assert left <= 1.0 and left >= 0.0
            assert right <= 1.0 and right >= 0.0
        except AssertionError:
            print("The speed parameters should be between 0.0 and 1.0")
            print("Setting both speed factors to default (1.0)")
            left = 1.0
            right = 1.0
        finally:
            self._left_speed = left
            self._right_speed = right

    def motorsMove(self, direction, speed=1.0):
        """ Command the motors to move
        Input: direction = ['left', 'right', 'forward', 'backward']
               speed = float => 0.0 to 1.0
        """
        try:
            assert speed <= 1.0
            assert speed >= 0.0
            assert direction in ['left', 'right', 'forward', 'backward']
        except AssertionError:
            # How to log errors?
            print("Speed parameters should be between 0.0 and 1.0")
            print("Not affecting any motor changes")

        left_speed_adj = speed * self._left_speed
        right_speed_adj = speed * self._right_speed

        if direction == 'forward':
            # Backward is forward on our robot
            self._left_motor.backward(left_speed_adj)
            self._right_motor.backward(right_speed_adj)
        elif direction == 'backward':
            # Forward is backward on our robot
            self._left_motor.forward(left_speed_adj)
            self._right_motor.forward(right_speed_adj)
        elif direction == 'left':
            self._left_motor.stop()
            self._right_motor.backward(right_speed_adj)
        elif direction == 'right':
            self._left_motor.backward(left_speed_adj)
            self._right_motor.stop()
        else:
            print("This shouldn't happen... seems like a direction \
                command slipped through")

    def motorsStop(self):
        """ Command the motors to stop """
        self._left_motor.stop()
        self._right_motor.stop()

    def boardStatus(self):
        """ The status of the SRV1 motor board
        Output: {'L Laser': Boolean, 'R Laser': Boolean, \
            'L Motor': Boolean, 'R Motor': Boolean}
        """
        return {'L Laser': self._left_laser.is_lit,
                'R Laser': self._right_laser.is_lit,
                'L Motor': self._left_motor.value,
                'R Motor': self._right_motor.value}

    def __str__(self):
        """ Let the __str__ method indicate the status of the board """
        return str(self.boardStatus())


if __name__ == "__main__":
    """ If this module is run as stand alone, do some tests """
    mb = SurveyorSRV1MotorBoard()

    mb.lasersOn()
    assert mb.boardStatus()['L Laser']
    assert mb.boardStatus()['R Laser']

    mb.lasersOff()
    assert not mb.boardStatus()['L Laser']
    assert not mb.boardStatus()['R Laser']

    mb.motorsMove('forward')
    assert mb.boardStatus()['L Motor'] == -1.0
    assert mb.boardStatus()['R Motor'] == -1.0

    mb.motorsMove('backward')
    assert mb.boardStatus()['L Motor'] == 1.0
    assert mb.boardStatus()['R Motor'] == 1.0

    mb.motorsMove('left')
    assert mb.boardStatus()['L Motor'] == 0.0
    assert mb.boardStatus()['R Motor'] == -1.0

    mb.motorsMove('right')
    assert mb.boardStatus()['L Motor'] == -1.0
    assert mb.boardStatus()['R Motor'] == 0.0

    print("All tests done!")
