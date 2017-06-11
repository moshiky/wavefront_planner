
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

    def __get_child_nodes(self, current_node):
        child_nodes = list()

        not_min_height = current_node[0] > 0
        not_min_width = current_node[1] > 0

        if not_min_height and not_min_width:
            new_node = list(current_node)
            new_node[0] -= 1
            new_node[1] -= 1
            if self.__world_map[new_node[0]][new_node[1]] != 1:
                child_nodes.append(new_node)

        if not_min_width:
            new_node = list(current_node)
            new_node[1] -= 1
            if self.__world_map[new_node[0]][new_node[1]] != 1:
                child_nodes.append(new_node)

        if not_min_height:
            new_node = list(current_node)
            new_node[0] -= 1
            if self.__world_map[new_node[0]][new_node[1]] != 1:
                child_nodes.append(new_node)

        return child_nodes

    def propagate_wavefront(self):
        self.__dfs_update(self.__goal_coordinates)

        print '********************************'
        print 'nodes=', self.__node_counter, 'updates=', self.__update_counter

    def __dfs_update(self, current_node):
        current_node_weight = self.__weights[current_node[0]][current_node[1]]

        # get child nodes
        child_nodes = self.__get_child_nodes(current_node)

        # update child nodes
        for node in child_nodes:
            self.__node_counter += 1
            node_weight = self.__weights[node[0]][node[1]]

            if node_weight > current_node_weight + 1:
                self.__weights[node[0]][node[1]] = current_node_weight + 1
                self.__update_counter += 1

                # print the cost of the end node
                print '********************************'
                print '\n'.join(str(x) for x in self.__weights)

                # call DFS search
                self.__dfs_update(node)

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
            [2, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 1, 0, 0, 0],
            [0, 0, 0, 1, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 3]
        ]
        self.__world_height = len(self.__world_map)
        self.__world_width = len(self.__world_map[0])

        # find start coordinates
        self.__start_coordinates = [0, 0]

        # find goal coordinates
        self.__goal_coordinates = [5, 7]


if __name__ == '__main__':
    start = time.time()

    planner = WavefrontPlanner('')
    planner.propagate_wavefront()
    optimal_path = planner.get_optimal_path()
    planner.store_path_image(optimal_path)

    end = time.time()
    print "Took %f seconds to run wavefront simulation" % (end - start)
