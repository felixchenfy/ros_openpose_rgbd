

class Timer(object):
    def __init__(self):
        self.t0 = time.time()

    def report_time(self, str_msg):
        t = time.time() - self.t0
        t = "{:.3f}".format(t)
        print("'{}' takes {} seconds".format(str_msg, t))

    def report_time_and_reset(self, str_msg):
        self.report_time(str_msg)
        self.reset()

    def reset(self):
        self.t0 = time.time()