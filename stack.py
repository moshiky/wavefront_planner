
class Stack:
    def __init__(self):
        self.__storage = list()

    def is_empty(self):
        return len(self.__storage) == 0

    def push(self, item):
        self.__storage.append(item)

    def top(self):
        if not self.is_empty():
            return self.__storage[-1]
        else:
            raise Exception("Stack is empty")

    def pop(self):
        last_item = self.top()
        self.__storage = self.__storage[:-1]
        return last_item
