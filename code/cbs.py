import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    l1, l2 = len(path1), len(path2)
    max_path = max(l1, l2)

    # deep copy of path1 and path2
    temp_path1, temp_path2 = [], []
    for pos in path1: 
        temp_path1.append(pos)
    for pos in path2:
        temp_path2.append(pos)

    if l1 > l2:
        for i in range(l1-l2):
            temp_path2.append(temp_path2[l2-1])
    else:
        for i in range(l2-l1):
            temp_path1.append(temp_path1[l1-1])

    for i in range(max_path):
         # detect vertex collision
        if get_location(temp_path1, i) == get_location(temp_path2, i):
            return [[get_location(temp_path1, i)], i]

        # detect edge collisions     
        if i < max_path-1:
             if get_location(temp_path1, i) == get_location(temp_path2, i+1) and get_location(temp_path2, i) == get_location(temp_path1, i+1):
                return [[get_location(temp_path1, i), get_location(temp_path1, i+1)], i+1]

    # no collision between two paths
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    collision_table = []
    for i in range(len(paths)):
        for j in range(i+1, len(paths)):
            detect_result = detect_collision(paths[i], paths[j])
            if detect_result != None:
                collision_table.append({'a1':i, 'a2':j, 'loc':detect_result[0], 'timestep':detect_result[1]})
    return collision_table


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    
    # vertex collision:
    if len(collision['loc']) == 1:
        first_constraint = {'agent':collision['a1'], 'loc':collision['loc'], 'timestep':collision['timestep'], 
                            'positive':0}
        second_constraint = {'agent':collision['a2'], 'loc':collision['loc'], 'timestep':collision['timestep'], 
                             'positive':0}
        return [first_constraint, second_constraint]

    # edge collision:
    else:
        first_constraint = {'agent':collision['a1'], 'loc':collision['loc'], 'timestep':collision['timestep'], 
                            'positive':0}
        second_constraint = {'agent':collision['a2'], 'loc':[collision['loc'][1], collision['loc'][0]], 
                             'timestep':collision['timestep'], 'positive':0}
        return [first_constraint, second_constraint]

def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    # Choose the agent randomly
    agent = ''
    if random.randint(0,1) == 0:
        agent = 'a1'
    else:
        agent = 'a2'

    # vertex collision:
    if len(collision['loc']) == 1:
        first_constraint = {'agent':collision[agent], 'loc':collision['loc'], 'timestep':collision['timestep'], 
                            'positive':1}
        second_constraint = {'agent':collision[agent], 'loc':collision['loc'], 'timestep':collision['timestep'], 
                             'positive':0}
        return [first_constraint, second_constraint]

    # edge collision:
    else:
        if agent == 'a1':
            first_constraint = {'agent':collision[agent], 'loc':collision['loc'], 'timestep':collision['timestep'], 
                                'positive':1}
            second_constraint = {'agent':collision[agent], 'loc':collision['loc'], 'timestep':collision['timestep'], 
                                 'positive':0}
            return [first_constraint, second_constraint]
        else:
            first_constraint = {'agent':collision[agent], 'loc':[collision['loc'][1], collision['loc'][0]], 
                                'timestep':collision['timestep'], 'positive':1}
            second_constraint = {'agent':collision[agent], 'loc':[collision['loc'][1], collision['loc'][0]], 
                                'timestep':collision['timestep'], 'positive':0}
            return [first_constraint, second_constraint]


# helper function: deep copy of parent attributes
def copy_parent(parent):
    copy = []
    for i in parent:
        copy.append(i)

    return copy

# helper function: 
def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is 1
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


# helper function: eliminate duplicates in path:
# example: [(5,5),(5,6),(5,6)] should be [(5,5),(5,6)]
def eliminate_duplicates(path):
    if path == None or len(path) <= 1:
        return path

    duplicate = path[len(path)-1]
    i = len(path)-2
    counter = 0
    while i >= 0:
        if path[i] == duplicate:
            counter += 1
        else:
            break
        i -= 1

    path = path[:len(path)-counter]
    return path



class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}

        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) > 0:
            curr = self.pop_node()     # firstly sorted with cost then sorted with #constraints 
            # no collision return solution
            if len(curr['collisions']) == 0:
                self.print_results(curr)
                return curr['paths']

            first_collision = curr['collisions'][0]
            constraints = []
            # standard CBS
            if disjoint == False:
                constraints = standard_splitting(first_collision)
            # CBS with disjoint splitting
            else:
                constraints = disjoint_splitting(first_collision)

            # for each constraint add a new child node 
            for constraint in constraints:
                # deep copy parent attributes to child 
                child_constarints = copy_parent(curr['constraints'])
                if constraint not in child_constarints:
                    child_constarints.append(constraint)
                child_paths = copy_parent(curr['paths'])
                constrainted_agent = constraint['agent']
                child = {'cost':0, 'constraints':child_constarints, 'paths':child_paths, 'collisions':[]}

                if constraint['positive'] == 0:
                    path = a_star(self.my_map, self.starts[constrainted_agent], self.goals[constrainted_agent], 
                                  self.heuristics[constrainted_agent], constrainted_agent, child['constraints'])
                    path = eliminate_duplicates(path)
                    if path is not None:
                        # replace the constrainted agent path by new path in parent paths
                        child['paths'][constrainted_agent] = path
                        child['collisions'] = detect_collisions(child['paths'])
                        child['cost'] = get_sum_of_cost(child['paths'])
                        self.push_node(child)
                else:
                    Add = True
                    # update all agents' path
                    violate_IDs = paths_violate_constraint(constraint, curr['paths'])

                    # replace the constrainted agent path by new path in parent paths
                    for each_agent in violate_IDs:
                        path = a_star(self.my_map, self.starts[each_agent], self.goals[each_agent], 
                                      self.heuristics[each_agent], each_agent, child['constraints'])
                        path = eliminate_duplicates(path)
                        if path is not None:
                            child['paths'][each_agent] = path
                        else:
                            Add = False
                            break

                    #update the constraint agent 
                    path = a_star(self.my_map, self.starts[constrainted_agent], self.goals[constrainted_agent], 
                                  self.heuristics[constrainted_agent], constrainted_agent, child['constraints'])
                    path = eliminate_duplicates(path)
                    if path is not None:
                        child['paths'][constrainted_agent] = path
                        child['collisions'] = detect_collisions(child['paths'])
                        child['cost'] = get_sum_of_cost(child['paths'])
                    else:
                        Add = False
                    if Add:
                        self.push_node(child)

        raise BaseException('No solutions')


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        print("Paths: ", node['paths'])