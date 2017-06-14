
import os
import time
from scipy import misc
from stack import Stack
from queue import Queue


class WavefrontPlanner:
    def __init__(self, map_file_path):
        # load map -> convert bitmap to 2D int array
        self.__load_map(map_file_path)

        # initiate weights
        self.__weights = list()
        self.__max_possible_steps = (self.__world_height * self.__world_width) + 1
        print "max steps:", self.__max_possible_steps

        for i in range(self.__world_height):
            self.__weights.append([self.__max_possible_steps] * self.__world_width)

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

    def propagate_wavefront_bfs(self):
        wavefront = Queue()
        wavefront.push(self.__goal_coordinates)

        print '#################################'
        if not self.__bfs_update(wavefront):
            print "start can't be reached"

        print 'nodes=', self.__node_counter, 'updates=', self.__update_counter
        print '#################################'

    def __bfs_update(self, wavefront):
        start_found = False
        visited = list()

        while not wavefront.is_empty() and not start_found:
            current_node = wavefront.pop()
            visited.append(current_node)

            current_node_weight = self.__weights[current_node[0]][current_node[1]]

            # get child nodes
            child_nodes = self.__get_child_nodes(current_node, wavefront)

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

                    # stop if start node reached
                    if node[0] == self.__start_coordinates[0] and node[1] == self.__start_coordinates[1]:
                        start_found = True

                # add node to wavefront
                if node not in visited:
                    wavefront.push(node)

        # return whether the start node found or not
        return start_found

    def propagate_wavefront_ids(self):
        limit = 0
        should_stop = False
        while not should_stop or limit == self.__max_possible_steps:
            limit += 1
            path_to_root = Stack()
            path_to_root.push(self.__goal_coordinates)
            print '>> limit=', limit
            should_stop = should_stop or self.__dfs_update(path_to_root, limit)

        print '#################################'
        print 'nodes=', self.__node_counter, 'updates=', self.__update_counter
        print '#################################'

    def __dfs_update(self, path_to_root, limit):
        if limit == 0:
            return False

        current_node = path_to_root.top()
        current_node_weight = self.__weights[current_node[0]][current_node[1]]

        # get child nodes
        child_nodes = self.__get_child_nodes(current_node, path_to_root)

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

                # stop if start node reached
                if node[0] == self.__start_coordinates[0] and node[1] == self.__start_coordinates[1]:
                    return True

            # call DFS search
            path_to_root.push(node)
            should_stop = self.__dfs_update(path_to_root, limit-1)
            path_to_root.pop()
            if should_stop:
                return True

        # if got here so shouldn't stop
        return False

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
        # read file
        with open(map_file_path) as image_file_obj:
            image_content = misc.imread(image_file_obj)

        self.__start_coordinates = None
        self.__goal_coordinates = None
        self.__world_height = image_content.shape[0]
        self.__world_width = image_content.shape[1]

        self.__world_map = list()
        for i in range(self.__world_height):
            self.__world_map.append(list())
            for j in range(self.__world_width):
                cell_value = 0  # white
                ccolor = image_content[i][j]
                if image_content[i][j] == [0, 0, 0]:    # black
                    cell_value = 1
                elif image_content[i][j] == [0, 255, 0] and self.__start_coordinates is None:    # blue
                    cell_value = 2
                    self.__start_coordinates = [i, j]
                elif image_content[i][j] == [255, 0, 0] and self.__goal_coordinates is None:    # red
                    cell_value = 3
                    self.__goal_coordinates = [i, j]
                elif image_content[i][j] != [255, 255, 255]:
                    raise Exception(
                        'cell [{i}, {j}] color not allowed: {color}'.format(i=i, j=j, color=image_content[i][j])
                    )
                self.__world_map[i].append(cell_value)

if __name__ == '__main__':
    start = time.time()

    planner = WavefrontPlanner(os.path.join(os.path.dirname(__file__), '..', 'files', 'map_with_goals.bmp'))
    # planner.propagate_wavefront_ids()
    planner.propagate_wavefront_bfs()
    optimal_path = planner.get_optimal_path()
    planner.store_path_image(optimal_path)

    end = time.time()
    print "Took %f seconds to run wavefront simulation" % (end - start)
