from pymongo import MongoClient
from datetime import datetime, timezone

def to_string(data):
    if isinstance(data, list):
        return ''.join(map(str, data))  # Join list elements into a single string
    return str(data)                    # Convert to string if it's not a list

def log_robot_data(db, robot_id, location_x, location_y, obstacles, confidence_score, scan_ranges, tasks_done):
    db.robot_logs.insert_one({
        "timestamp": datetime.now(timezone.utc),
        "robot_id": robot_id,
        "X": location_x, "Y": location_y,
        "obstacles": obstacles,
        "confidence_score": confidence_score,
        "scan_ranges": scan_ranges,
        "tasks_done": tasks_done
    })

def update_dynamic_map(db, environment_id, new_data):
    existing_map = db.dynamic_map.find_one({"environment_id": environment_id})
    if (existing_map):
        new_data_str = to_string(new_data)
        existing_map_str = to_string(existing_map['map_data'])
        if (new_data_str not in existing_map_str):
            updated_map_data = existing_map['map_data'] + new_data
            db.dynamic_map.update_one(
                {"environment_id": environment_id},
                {"$set": {"map_data": updated_map_data, "last_updated": datetime.now(timezone.utc)}}
            )
            print(f"Dynamic map updated for environment ID: {environment_id}")
        else:
            print(f"Dynamic map for environment ID: {environment_id} is already up-to-date.")
    else:
        db.dynamic_map.insert_one({
            "environment_id": environment_id,
            "map_data": new_data,
            "last_updated": datetime.now(timezone.utc)
        })
        print("Environment ID not found.")
        print(f"Initial dynamic map added for environment ID: {environment_id}")        