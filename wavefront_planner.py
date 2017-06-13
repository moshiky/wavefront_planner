
import time
from stack import Stack


class WavefrontPlanner:
    def __init__(self, map_file_path):
        # load map -> convert bitmap to 2D int array
        self.__load_map(map_file_path)

        # initiate weights
        self.__weights = list()
        max_possible_steps = (self.__world_height * self.__world_width) + 1
        print "max steps:", max_possible_steps

        for i in range(self.__world_height):
            self.__weights.append([max_possible_steps] * self.__world_width)

        # initiate goal weight
        self.__weights[self.__goal_coordinates[0]][self.__goal_coordinates[1]] = 2

        # initiate nodes counter
        self.__node_counter = 1

        # initiate update counter
        self.__update_counter = 0

    def __is_valid_child(self, child_node, path_to_root):
        return \
            child_node not in path_to_root \
            and 0 <= child_node[0] < self.__world_height \
            and 0 <= child_node[1] < self.__world_width \
            and self.__world_map[child_node[0]][child_node[1]] != 1

    def __get_child_nodes(self, current_node, path_to_root):
        child_nodes = list()

        for i in range(-1, 2):
            for j in range(-1, 2):
                new_node = list(current_node)
                new_node[0] += i
                new_node[1] += j
                if self.__is_valid_child(new_node, path_to_root):
                    child_nodes.append(new_node)

        return child_nodes

    def propagate_wavefront(self):
        limit = 0
        should_stop = False
        while not should_stop:
            path_to_root = Stack()
            path_to_root.push(self.__goal_coordinates)
            should_stop = should_stop and self.__dfs_update(path_to_root, limit)
            limit += 1

        print '********************************'
        print 'nodes=', self.__node_counter, 'updates=', self.__update_counter

    def __dfs_update(self, path_to_root, limit):
        current_node = path_to_root.top()
        current_node_weight = self.__weights[current_node[0]][current_node[1]]

        # get child nodes
        child_nodes = self.__get_child_nodes(current_node, path_to_root)

        # update child nodes
        for node in child_nodes:
            self.__node_counter += 1
            node_weight = self.__weights[node[0]][node[1]]

            if node_weight > current_node_weight + 1:
                if limit == 0:
                    return False

                self.__weights[node[0]][node[1]] = current_node_weight + 1
                self.__update_counter += 1

                # print the cost of the end node
                print '********************************'
                print '\n'.join(str(x) for x in self.__weights)

                # call DFS search
                path_to_root.push(node)
                self.__dfs_update(path_to_root)
                path_to_root.pop()

    def get_optimal_path(self):
        # find optimal path
        return 0

    def store_path_image(self, path):
        pass

    def __load_map(self, map_file_path):
        """
        Find start and goal coordinates -> empty=0 wall=1 start=2 goal=3
        :param map_file_path:
        :return:
        """
        self.__world_map = [
            [0, 3, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 0, 0, 0],
            [0, 0, 0, 1, 1, 0, 0, 0],
            [0, 0, 2, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0]
        ]
        self.__world_height = len(self.__world_map)
        self.__world_width = len(self.__world_map[0])

        # search start and goal coordinates
        self.__start_coordinates = None
        self.__goal_coordinates = None

        i = 0
        while (self.__start_coordinates is None or self.__goal_coordinates is None) and i < self.__world_height:
            j = 0
            while (self.__start_coordinates is None or self.__goal_coordinates is None) and j < self.__world_width:
                if self.__world_map[i][j] == 2:
                    self.__start_coordinates = [i, j]
                elif self.__world_map[i][j] == 3:
                    self.__goal_coordinates = [i, j]
                j += 1

            i += 1

if __name__ == '__main__':
    start = time.time()

    planner = WavefrontPlanner('')
    planner.propagate_wavefront()
    optimal_path = planner.get_optimal_path()
    planner.store_path_image(optimal_path)

    end = time.time()
    print "Took %f seconds to run wavefront simulation" % (end - start)
