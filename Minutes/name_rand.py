import random
from datetime import datetime
random.seed(datetime.now())
Name_List = "Austin Bryce Drew Matt Prashant Steve Taylor Tyler"
Name_List = Name_List.split()
print Name_List
random.shuffle(Name_List)
print Name_List

