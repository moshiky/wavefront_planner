
class DistributedBFS:

    def __init__(self, world_map, weights):
        self.__world_map = world_map
        self.__weights = weights

    def update_cell(self, top_left_i, top_left_j, cell_size, propagation_direction):
        """
        update the cell
        :param top_left_i:
        :param top_left_j:
        :param cell_size:
        :param propagation_direction: (i_factor, j_factor)

              7 0 1
           -> 6 C 2 <-
              5 4 3

        :return:
        """
        # copy cell locally

        # update first edge from caller cell

        # continue edge to all cell

        # copy cell to original
