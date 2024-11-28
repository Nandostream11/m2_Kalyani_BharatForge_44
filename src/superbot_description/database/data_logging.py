from pymongo import MongoClient
from datetime import datetime, timezone

def log_robot_data(db, robot_id, location, obstacles, tasks_completed):
    db.robot_logs.insert_one({
        "robot_id": robot_id,
        "timestamp": datetime.now(timezone.utc),
        "location": location,
        "obstacles": obstacles,
        "tasks_completed": tasks_completed
    })

def log_dynamic_map(db, environment_id, map_data):
    db.dynamic_map.insert_one({
        "environment_id": environment_id,
        "map_data": map_data,
        "last_updated": datetime.now(timezone.utc)
    })