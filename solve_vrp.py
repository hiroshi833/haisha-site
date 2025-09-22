# solve_vrp_time_simple.py
# Usage:
# python solve_vrp_time_simple.py input.json
#
# This version:
# - uses straight-line distance (haversine) as "直線距離"
# - assumes constant speed (speed_kmh -> convert distance to seconds)
# - builds a time matrix (seconds) and uses OR-Tools CVRP with capacity
# - enforces capacity (3 per vehicle) and attempts to ensure all vehicles used
# - does a small TSP reordering per vehicle for nicer order
# - prints JSON result with vehicle assignments and estimated total time (seconds)

import json, sys, math
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# ---------------------
# util distance/time
# ---------------------
def haversine_m(lat1, lon1, lat2, lon2):
    # returns meters (great-circle distance)
    R = 6371000.0
    phi1 = math.radians(lat1); phi2 = math.radians(lat2)
    dphi = math.radians(lat2-lat1); dlambda = math.radians(lon2-lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def travel_time_seconds(lat1, lon1, lat2, lon2, speed_kmh):
    # straight-line distance (meters) / speed -> seconds
    meters = haversine_m(lat1, lon1, lat2, lon2)
    # convert speed km/h to m/s
    speed_ms = (speed_kmh * 1000.0) / 3600.0
    if speed_ms <= 0:
        speed_ms = 11.11  # fallback ~40 km/h
    return int(meters / speed_ms + 0.5)

# ---------------------
# build time matrix (seconds)
# ---------------------
def build_time_matrix(depot, customers, speed_kmh):
    points = [depot] + [(c['lat'], c['lng']) for c in customers]
    n = len(points)
    mat = [[0]*n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i == j:
                mat[i][j] = 0
            else:
                mat[i][j] = travel_time_seconds(points[i][0], points[i][1], points[j][0], points[j][1], speed_kmh)
    return mat

# ---------------------
# OR-Tools CVRP (time-based matrix used as distance cost)
# ---------------------
def solve_cvrp_time(time_matrix, vehicle_count, capacity, demands, service_times, vehicle_max_time):
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), vehicle_count, 0)
    routing = pywrapcp.RoutingModel(manager)

    # transit (travel time) callback
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # travel time + service time at from_node
        return time_matrix[from_node][to_node]
    transit_idx = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_idx)

    # capacity constraint
    def demand_callback(index):
        node = manager.IndexToNode(index)
        return demands[node]
    demand_idx = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(demand_idx, 0, [capacity]*vehicle_count, True, 'Capacity')

    # time dimension: we incorporate service time separately by adding to the node cumul via unary callback
    def time_with_service_callback(from_index, to_index):
        # we will compute travel time + service time at from_node
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node] + service_times[from_node]
    time_ws_idx = routing.RegisterTransitCallback(time_with_service_callback)
    routing.AddDimension(
        time_ws_idx,
        0,  # slack
        vehicle_max_time,  # per vehicle max cumulative time (seconds)
        True,  # start cumul to zero
        'Time'
    )
    time_dim = routing.GetDimensionOrDie('Time')

    # Do not allow dropping nodes: all customers must be visited
    # (OR-Tools allows setting penalties to drop; we won't)

    # search params
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 10

    solution = routing.SolveWithParameters(search_parameters)
    if not solution:
        return None

    routes = []
    for v in range(vehicle_count):
        idx = routing.Start(v)
        route = []
        while not routing.IsEnd(idx):
            node = manager.IndexToNode(idx)
            if node != 0:
                route.append(node-1)  # customer index
            idx = solution.Value(routing.NextVar(idx))
        routes.append(route)
    return routes

# small TSP reordering for cluster (brute force, cluster small)
def solve_tsp_for_cluster(cluster_indices, time_matrix):
    if not cluster_indices:
        return []
    # map: depot=0, cluster nodes = idx+1
    idx_map = [0] + [i+1 for i in cluster_indices]
    n = len(idx_map)
    import itertools
    best = None
    for perm in itertools.permutations(range(1, n)):
        order = [0] + list(perm) + [0]
        cost = 0
        for i in range(len(order)-1):
            a = idx_map[order[i]]
            b = idx_map[order[i+1]]
            cost += time_matrix[a][b]
        if best is None or cost < best[0]:
            best = (cost, order)
    # convert best order to original customer indices
    ordered_customers = []
    for i in best[1]:
        if i == 0:
            continue
        ordered_customers.append(idx_map[i]-1)
    return ordered_customers

def ensure_all_vehicles_used(routes, customer_count, vehicle_count):
    routes = [list(r) for r in routes]
    # while exists empty route, move from largest (if possible)
    while sum(1 for r in routes if len(r) == 0) > 0:
        # find biggest with >1
        idx_big = max(range(len(routes)), key=lambda i: len(routes[i]))
        if len(routes[idx_big]) <= 1:
            # cannot move further
            break
        idx_empty = next(i for i,r in enumerate(routes) if len(r) == 0)
        moved = routes[idx_big].pop()
        routes[idx_empty].append(moved)
    return routes

# ---------------------
# main
# ---------------------
def main():
    if len(sys.argv) < 2:
        print("Usage: python solve_vrp_time_simple.py input.json")
        sys.exit(1)
    with open(sys.argv[1], 'r', encoding='utf-8') as f:
        data = json.load(f)

    depot = data['depot']
    vehicle_count = int(data.get('cars', 1))
    customers = data['customers']
    speed_kmh = float(data.get('speed_kmh', 40.0))
    max_time_minutes = int(data.get('max_time_minutes', 180))
    service_time_minutes = float(data.get('service_time_minutes', 5.0))

    if len(customers) < vehicle_count:
        print("Error: customers fewer than vehicles. Need at least one customer per vehicle.")
        sys.exit(1)

    # build time matrix (seconds)
    time_mat = build_time_matrix(depot, customers, speed_kmh)

    # demands: depot=0, customers=1 each
    demands = [0] + [1]*len(customers)
    # service times (seconds): depot=0
    service_times = [0] + [int(service_time_minutes*60 + 0.5) for _ in customers]
    capacity = 3
    vehicle_max_time = int(max_time_minutes*60)

    # solve CVRP (time matrix used as arc cost)
    routes = solve_cvrp_time(time_mat, vehicle_count, capacity, demands, service_times, vehicle_max_time)
    if routes is None:
        print("No solution found by OR-Tools")
        sys.exit(1)

    # ensure all vehicles used
    routes = ensure_all_vehicles_used(routes, len(customers), vehicle_count)

    # reorder each cluster by TSP (time)
    final = []
    vehicle_total_time = []
    for r in routes:
        if len(r) == 0:
            final.append([])
            vehicle_total_time.append(0)
        else:
            order = solve_tsp_for_cluster(r, time_mat)
            final.append(order)
            # compute total time: depot->first + between + last->depot + service times
            total = 0
            prev = 0
            for idx in order:
                total += time_mat[prev][idx+1]  # prev to customer
                total += service_times[idx+1]   # service at customer
                prev = idx+1
            total += time_mat[prev][0]  # last back to depot
            vehicle_total_time.append(total)

    # build output
    out = {"result": []}
    for i, cluster in enumerate(final):
        names = [customers[idx]['name'] for idx in cluster]
        out["result"].append({
            "vehicle": i+1,
            "customers": names,
            "estimated_time_seconds": vehicle_total_time[i]
        })

    print(json.dumps(out, ensure_ascii=False, indent=2))

if __name__ == "__main__":
    main()
