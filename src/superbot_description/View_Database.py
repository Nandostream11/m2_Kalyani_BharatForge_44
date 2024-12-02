from pymongo import MongoClient

def connect_to_db():
    client = MongoClient('mongodb://localhost:27017/')
    db = client['robot_navigation']
    return db

def fetch_robot_logs(db):
    logs = db.robot_logs.find()
    for log in logs:
        print(log)

def fetch_dynamic_map(db):
    maps = db.dynamic_map.find()
    for map_entry in maps:
        print(map_entry)

if __name__ == "__main__":
    db = connect_to_db()
    print("Robot Logs:")
    fetch_robot_logs(db)
    print("\nDynamic Maps:")
    fetch_dynamic_map(db)