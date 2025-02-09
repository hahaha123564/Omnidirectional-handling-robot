import pickle
file_path = 'home/spark/spark_noetic/src/3rd_app/auto_match_24/auto_match/config/'
with open(file_path, 'rb') as file:
    data = pickle.load(file)
print(data)
