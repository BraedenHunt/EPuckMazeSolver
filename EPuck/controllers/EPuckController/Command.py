class Command:
    initialized = False

    def is_finished(self):
        return False

    def cancel(self):
        pass

    def update(self, time):
        pass

    def initialize(self, time):
        self.initialized = True
        pass
