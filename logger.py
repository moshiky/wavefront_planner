
import os
import time


class Logger:

    def __init__(self):
        self.__log_file_name = \
            os.path.join(
                os.path.dirname(__file__),
                'logs',
                'log__{timestamp}.log'.format(timestamp=time.strftime('%d_%m_%Y__%H_%M_%S'))
            )
        self.log('Logger created', should_print=False)

    def error(self, msg):
        self.log('ERROR: {msg}'.format(msg=msg))

    def log(self, msg, should_print=True):
        formatter_msg = '[{timestamp}] >> {msg}'.format(timestamp=time.strftime('%d/%m/%Y %H:%M:%S'), msg=msg)
        with open(self.__log_file_name, 'ab') as log_file:
            log_file.write('{msg}\n'.format(msg=formatter_msg))

        if should_print:
            self.print_msg(msg)

    def print_msg(self, msg):
        print '[{timestamp}] >> {msg}'.format(timestamp=time.strftime('%d/%m/%Y %H:%M:%S'), msg=msg)
