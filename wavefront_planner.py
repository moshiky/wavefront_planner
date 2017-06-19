
import os
import numpy as np
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

        # initiate maximal weight
        self.__max_weight = 0

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
        self.__max_weight = 0

        while not wavefront.is_empty() and not start_found:
            current_node = wavefront.pop()
            visited.append(current_node)

            current_node_weight = self.__weights[current_node[0]][current_node[1]]
            if current_node_weight > self.__max_weight:
                self.__max_weight = current_node_weight
                self.__logger.log('depth= {depth}'.format(depth=current_node_weight))

            # get child nodes
            child_nodes = self.__get_child_nodes(current_node, list())

            # update child nodes
            for node in child_nodes:
                self.__node_counter += 1
                node_weight = self.__weights[node[0]][node[1]]
                move_direction = WavefrontPlanner.__get_direction(current_node, node)
                if 0 in move_direction:
                    addition = 1
                else:
                    addition = np.sqrt(2)

                if node_weight > current_node_weight + addition:

                    self.__weights[node[0]][node[1]] = current_node_weight + addition
                    self.__update_counter += 1

                    # stop if start node reached
                    if node[0] == self.__start_coordinates[0] and node[1] == self.__start_coordinates[1]:
                        start_found = True

                # add node to wavefront
                if node not in visited and node not in wavefront:
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

    def __get_next_node(self, current_node, current_direction=None):
        min_value = self.__max_possible_steps
        best_nodes = list()

        for i in range(-1, 2):
            for j in range(-1, 2):
                node_weight = self.__weights[current_node[0]+i][current_node[1]+j]
                if node_weight <= min_value:
                    if node_weight < min_value:
                        min_value = node_weight
                        best_nodes = list()

                    best_nodes.append([current_node[0]+i, current_node[1]+j])

        # if current_direction is not None:
        #     current_direction_node = [current_node[0] + current_direction[0], current_node[1] + current_direction[1]]
        #     if self.__weights[current_direction_node[0]][current_direction_node[1]] == best_nodes:
        #         best_nodes = current_direction_node

        closest_node = best_nodes[0]
        minimal_distance = self.__get_goal_distance(closest_node)
        for node in best_nodes:
            goal_distance = self.__get_goal_distance(node)
            if goal_distance < minimal_distance:
                minimal_distance = goal_distance
                closest_node = node

        return closest_node

    def __get_goal_distance(self, node):
        return np.sqrt(
            np.square(self.__goal_coordinates[0] - node[0])
            + np.square(self.__goal_coordinates[1] - node[1])
        )

    def get_optimal_path(self):
        path = list()

        current_node = self.__start_coordinates
        while current_node != self.__goal_coordinates:
            path.append(current_node)
            current_node = self.__get_next_node(current_node)

        return path[1:]

    @staticmethod
    def __get_direction(previous_node, current_node):
        return [current_node[0]-previous_node[0], current_node[1]-previous_node[1]]

    def get_optimal_smooth_path(self):
        path = list()

        current_node = self.__start_coordinates
        previous_node = current_node
        while current_node != self.__goal_coordinates:
            path.append(current_node)
            new_node = self.__get_next_node(current_node, WavefrontPlanner.__get_direction(previous_node, current_node))
            previous_node = current_node
            current_node = new_node

        return path[1:]

    def save_path_map(self, path, algorithm_name):
        # duplicate map
        new_map = list()
        for row in self.__world_map:
            new_map.append(list(row))

        # color path
        for node in path:
            new_map[node[0]][node[1]] = 4   # green

        # convert map to np array with RGB color scheme
        for i in range(self.__world_height):
            new_map[i] = np.array(map(WavefrontPlanner.__get_rgb_color, new_map[i]))
        new_map = np.array(new_map)

        # read file
        self.__logger.log('saving map into file')
        with open(algorithm_name + '_output.bmp', 'wb') as image_file_obj:
            misc.imsave(image_file_obj, new_map)

    def save_propagation_map(self, algorithm_name):
        # duplicate map
        new_map = list()
        for row in self.__world_map:
            new_map.append(list(row))

        # calculate color change interval
        color_change_interval = int((255*2) / (self.__max_weight-2))

        # convert map to np array with RGB color scheme
        for i in range(self.__world_height):
            new_map[i] = np.array(map(WavefrontPlanner.__get_rgb_color, new_map[i]))
            new_map[i] = self.__get_propagation_row_color(new_map[i], i, color_change_interval)

        new_map = np.array(new_map)

        # read file
        self.__logger.log('saving map into file')
        with open(algorithm_name + '_propagation_output.bmp', 'wb') as image_file_obj:
            misc.imsave(image_file_obj, new_map)

    def __get_propagation_row_color(self, original_row, row_index, color_change_interval):
        new_row = list()

        for j in range(self.__world_width):
            new_cell = original_row[j]
            if (new_cell == [255, 255, 255]).all():   # we color just white cells
                if self.__weights[row_index][j] < self.__max_possible_steps:    # cell was updated
                    color_index = (self.__weights[row_index][j] - 2) * color_change_interval
                    new_cell = [
                        (255 - color_index) if color_index < 255 else 0,
                        color_index if color_index < 255 else (510 - color_index),
                        (color_index - 255) if 255 <= color_index else 0
                    ]

            new_row.append(new_cell)

        return new_row

    @staticmethod
    def __get_rgb_color(cell_value):
        if cell_value == 0:         # empty cell
            return [255, 255, 255]  # white
        elif cell_value == 1:       # wall
            return [0, 0, 0]        # black
        elif cell_value == 2:       # start
            return [0, 0, 255]      # blue
        elif cell_value == 3:       # goal
            return [255, 0, 0]      # red
        elif cell_value == 4:       # path
            return [0, 255, 0]      # green
        else:
            raise Exception('unsupported cell type: {cell_value}'.format(cell_value=cell_value))

    def __load_map(self, map_file_path):
        """
        Find start and goal coordinates -> empty=0 wall=1 start=2 goal=3
        :param map_file_path:
        :return:
        """
        self.__logger.log('loading {file_name}'.format(file_name=map_file_path))

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
        with open(map_file_path, 'rb') as image_file_obj:
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
                    self.__logger.log(
                        'assuming cell is wall: [{i}, {j}], color={color}'.format(i=i, j=j, color=image_content[i][j]),
                        should_print=False
                    )
                    cell_value = 1
                self.__world_map[i].append(cell_value)

        if self.__start_coordinates is None or self.__goal_coordinates is None:
            raise Exception(
                'start or goal cells not found'
            )


if __name__ == '__main__':
    logger = Logger()

    map_file_name = 'maze_with_goals.bmp'

    bfs_planner = WavefrontPlanner(os.path.join(os.path.dirname(__file__), '..', 'files', map_file_name), logger)
    bfs_planner.propagate_wavefront_bfs()
    bfs_planner.save_propagation_map('bfs')
    optimal_path = bfs_planner.get_optimal_path()
    bfs_planner.save_path_map(optimal_path, 'bfs')


