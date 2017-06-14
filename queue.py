
class Queue:
    def __init__(self):
        self.__storage = list()

    def is_empty(self):
        return len(self.__storage) == 0

    def push(self, item):
        self.__storage.append(item)

    def first(self):
        if not self.is_empty():
            return self.__storage[0]
        else:
            raise Exception("Queue is empty")

    def pop(self):
        first_item = self.first()
        self.__storage = self.__storage[1:]
        return first_item

    def __contains__(self, item):
        return item in self.__storage
