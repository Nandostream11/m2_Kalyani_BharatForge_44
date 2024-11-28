from pymongo import MongoClient
from datetime import datetime, timezone

def update_dynamic_map(db, environment_id, new_data):
    existing_map = db.dynamic_map.find_one({"environment_id": environment_id})
    if existing_map:
        updated_map_data = existing_map['map_data'] + new_data
        db.dynamic_map.update_one(
            {"environment_id": environment_id},
            {"$set": {"map_data": updated_map_data, "last_updated": datetime.now(timezone.utc)}}
        )
    else:
        print("Environment ID not found.")