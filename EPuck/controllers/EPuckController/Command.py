class Command:
    def is_finished(self):
        return False

    def cancel(self):
        pass

    def update(self, time):
        pass

    def initialize(self, time):
        pass
