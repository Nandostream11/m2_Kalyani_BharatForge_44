from pymongo import MongoClient

def connect_to_db():
    client = MongoClient('mongodb://localhost:27017/')
    db = client['robot_navigation']                     #Name of the database; 
    print("Connected to MongoDB")
    return db

if __name__ == "__main__":
    db = connect_to_db()
    print("Connected to MongoDB")

'''
terminal code to display the data in the database(run below code or run View_Database.py):
mongosh

use robot_navigation
db.robot_logs.find().pretty()
-------------------------------------
db.dynamic_map.find().pretty()
-------------------------------------
'''