from pymongo import MongoClient

def connect_to_db():
    client = MongoClient('mongodb://localhost:27017/')
    db = client['robot_navigation']
    return db

if __name__ == "__main__":
    db = connect_to_db()
    print("Connected to MongoDB")
