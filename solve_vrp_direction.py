import json
import math

# 拠点の緯度経度
BASE = {"lat": 35.0044801, "lng": 135.7736121}

# 女の子データと車台数をinput.jsonで読み込む
def load_input(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    car_count = data["car_count"]
    girls = data["girls"]
    return car_count, girls

# 拠点からの距離（直線距離）を計算
def distance(a, b):
    dx = a["lat"] - b["lat"]
    dy = a["lng"] - b["lng"]
    return math.sqrt(dx*dx + dy*dy)

# 方角（簡易）を計算
def angle_from_base(girl):
    dx = girl["lat"] - BASE["lat"]
    dy = girl["lng"] - BASE["lng"]
    return math.atan2(dy, dx)  # ラジアン

# 車への割り当て
def assign_cars(car_count, girls):
    # 拠点から遠い順にソート
    girls_sorted = sorted(girls, key=lambda g: distance(BASE, g), reverse=True)

    # 初期の空車リスト
    cars = [[] for _ in range(car_count)]

    # まず全車に最低1人割り当てる
    for i in range(min(car_count, len(girls_sorted))):
        cars[i].append(girls_sorted[i])

    # 残りの女の子を方角順に振り分け
    for g in girls_sorted[car_count:]:
        # 拠点からの角度で最も近い車に追加（3人まで）
        best_car_idx = None
        min_angle_diff = float("inf")
        for idx, car in enumerate(cars):
            if len(car) >= 3:
                continue
            # 車の平均角度（拠点から）
            if len(car)==0:
                avg_angle = 0
            else:
                avg_angle = sum(angle_from_base(c) for c in car)/len(car)
            diff = abs(avg_angle - angle_from_base(g))
            if diff < min_angle_diff:
                min_angle_diff = diff
                best_car_idx = idx
        if best_car_idx is None:
            # どの車も満員なら、追加できる車に入れる
            for idx, car in enumerate(cars):
                if len(car) < 3:
                    best_car_idx = idx
                    break
        cars[best_car_idx].append(g)

    # 結果整形（名前と推定時間）
    result = []
    for idx, car in enumerate(cars):
        if len(car)==0:
            estimated_time = 0
        else:
            # 単純に往復距離を距離=時間換算
            route_distance = sum(distance(BASE, g) for g in car)*2
            estimated_time = int(route_distance*100)  # 適当な係数で秒に
        result.append({
            "vehicle": idx+1,
            "customers": [g["name"] for g in car],
            "estimated_time_seconds": estimated_time
        })
    return result

def main():
    car_count, girls = load_input("input.json")
    res = assign_cars(car_count, girls)
    output = {"result": res}
    print(json.dumps(output, ensure_ascii=False, indent=2))

if __name__=="__main__":
    main()
