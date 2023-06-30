class TimingAlignment:
    """
    Execute actions every n number of time-steps.
    This prevents the program from executing performance intensive tasks each step,
     which would drastically reduce the simulation speed of the program.
    """

    def __init__(self, interval: int):
        """
        :param interval: Number of time-steps to wait before executing the action.
        """
        self.__current_step = 0
        self.interval = interval

    def increase(self):
        self.__current_step += 1
        if self.__current_step == self.interval:
            self.__current_step = 0

    @property
    def check(self):
        return self.__current_step == 0
