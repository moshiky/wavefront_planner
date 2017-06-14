
import os
import time
from scipy import misc
from stack import Stack
from queue import Queue
from logger import Logger


class WavefrontPlanner:
    def __init__(self, map_file_path, logger_p):
        # create logger
        self.__logger = logger_p

        # load map -> convert bitmap to 2D int array
        self.__load_map(map_file_path)

        # initiate weights
        self.__weights = list()
        self.__max_possible_steps = (self.__world_height * self.__world_width) + 1
        self.__logger.log("max steps= {max_steps}".format(max_steps=self.__max_possible_steps))

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
        self.__logger.log('###### running BFS ######')
        wavefront = Queue()
        wavefront.push(self.__goal_coordinates)

        is_goal_found = self.__bfs_update(wavefront)
        self.__logger.log('###### BFS results ######')
        if not is_goal_found:
            self.__logger.log("start can't be reached")

        self.__logger.log('nodes= {nodes}, updates= {updates}'.format(
            nodes=self.__node_counter, updates=self.__update_counter)
        )
        self.__logger.log('###### BFS end ######')

    def __bfs_update(self, wavefront):
        start_found = False
        visited = list()
        max_weight = 0

        while not wavefront.is_empty() and not start_found:
            current_node = wavefront.pop()
            # self.__logger.log(str(current_node))
            visited.append(current_node)

            current_node_weight = self.__weights[current_node[0]][current_node[1]]
            if current_node_weight > max_weight:
                max_weight = current_node_weight
                self.__logger.log('depth= {depth}'.format(depth=current_node_weight))

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
                    # self.__logger.log('********************************')
                    # self.__logger.log('\n'.join(str(x) for x in self.__weights))

                    # stop if start node reached
                    if node[0] == self.__start_coordinates[0] and node[1] == self.__start_coordinates[1]:
                        start_found = True

                # add node to wavefront
                if node not in visited:
                    wavefront.push(node)

        # return whether the start node found or not
        return start_found

    def propagate_wavefront_ids(self):
        self.__logger.log('###### running IDS ######')
        limit = 0
        should_stop = False
        while not should_stop and limit < self.__max_possible_steps:
            limit += 1
            path_to_root = Stack()
            path_to_root.push(self.__goal_coordinates)
            self.__logger.log('limit= {limit}'.format(limit=limit))
            should_stop = should_stop or self.__dfs_update(path_to_root, limit)

        self.__logger.log('###### IDS results ######')
        if not should_stop:
            self.__logger.log("start can't be reached")

        self.__logger.log('nodes= {nodes}, updates= {updates}'.format(
            nodes=self.__node_counter, updates=self.__update_counter)
        )
        self.__logger.log('###### IDS end ######')

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
                # self.__logger.log('********************************')
                # self.__logger.log('\n'.join(str(x) for x in self.__weights))

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
        parsed_data_file_path = map_file_path.split('.')[0] + '.data'
        if not os.path.exists(parsed_data_file_path):
            self.__parse_file(map_file_path)
            self.__store_parsed_data(parsed_data_file_path)
        else:
            self.__load_parsed_data_file(parsed_data_file_path)

        self.__logger.log('start= {start}, goal={goal}'.format(
            start=self.__start_coordinates, goal=self.__goal_coordinates)
        )

        self.__logger.log('map loaded successfully')

    def __store_parsed_data(self, parsed_data_file_path):
        map_text = \
            str(self.__start_coordinates[0]) + '_' + \
            str(self.__start_coordinates[1]) + '_' + \
            str(self.__goal_coordinates[0]) + '_' + \
            str(self.__goal_coordinates[1]) + '\n'

        for row in self.__world_map:
            for cell in row:
                map_text += str(cell) + ' '
            map_text = map_text[:-1]
            map_text += '\n'
        map_text = map_text[:-1]

        with open(parsed_data_file_path, 'wb') as parsed_data_file:
            parsed_data_file.write(map_text)

    def __load_parsed_data_file(self, parsed_data_file_path):
        with open(parsed_data_file_path, 'rb') as parsed_data_file:
            map_text = parsed_data_file.read()

        map_text_rows = map_text.split('\n')
        self.__world_map = list()
        for row in map_text_rows[1:]:
            self.__world_map.append(map(int, row.split(' ')))

        first_row_parts = map(int, map_text_rows[0].split('_'))
        self.__start_coordinates = [first_row_parts[0], first_row_parts[1]]
        self.__goal_coordinates = [first_row_parts[2], first_row_parts[3]]

        self.__world_height = len(self.__world_map)
        self.__world_width = len(self.__world_map[0])

    def __parse_file(self, map_file_path):
        # read file
        self.__logger.log('loading map file')
        with open(map_file_path) as image_file_obj:
            image_content = misc.imread(image_file_obj)

        self.__start_coordinates = None
        self.__goal_coordinates = None
        self.__world_height = image_content.shape[0]
        self.__world_width = image_content.shape[1]

        self.__world_map = list()
        self.__logger.log('parsing map')
        for i in range(self.__world_height):
            if (i + 1) % 100 == 0:
                self.__logger.log('row #' + str(i + 1))

            self.__world_map.append(list())
            for j in range(self.__world_width):
                if (image_content[i][j] == [0, 0, 0]).all():  # black
                    cell_value = 1
                elif (image_content[i][j] == [0, 0, 255]).all():  # blue
                    if self.__start_coordinates is None:
                        cell_value = 2
                        self.__start_coordinates = [i, j]
                    else:
                        cell_value = 0
                elif (image_content[i][j] == [255, 0, 0]).all():  # red
                    if self.__goal_coordinates is None:
                        cell_value = 3
                        self.__goal_coordinates = [i, j]
                    else:
                        cell_value = 0
                elif (image_content[i][j] == [255, 255, 255]).all():  # white
                    cell_value = 0
                else:
                    raise Exception(
                        'cell [{i}, {j}] color not allowed: {color}'.format(i=i, j=j, color=image_content[i][j])
                    )
                self.__world_map[i].append(cell_value)

        if self.__start_coordinates is None or self.__goal_coordinates is None:
            raise Exception(
                'start or goal cells not found'
            )


if __name__ == '__main__':
    logger = Logger()

    bfs_planner = WavefrontPlanner(os.path.join(os.path.dirname(__file__), '..', 'files', 'map_with_goals.bmp'), logger)
    bfs_planner.propagate_wavefront_bfs()

    ids_planner = WavefrontPlanner(os.path.join(os.path.dirname(__file__), '..', 'files', 'map_with_goals.bmp'), logger)
    ids_planner.propagate_wavefront_ids()

    # optimal_path = planner.get_optimal_path()
    # planner.store_path_image(optimal_path)
