from pymongo import MongoClient
from datetime import datetime, timezone

def log_robot_data(db, robot_id, location_x, location_y, obstacles, confidence_score, tasks_done):
    db.robot_logs.insert_one({
        "timestamp": datetime.now(timezone.utc),
        "robot_id": robot_id,
        "X": location_x, "Y": location_y,
        "obstacles": obstacles,
        "confidence_score": confidence_score,
        "tasks_done": tasks_done
    })

def log_dynamic_map(db, environment_id, map_data):
    db.dynamic_map.insert_one({
        "environment_id": environment_id,
        "map_data": map_data,
        "last_updated": datetime.now(timezone.utc)
    })