import numpy as np 
import pandas as pd

def write_to_csv(name, data): 
    path = 'data/'+str(name)+'.csv'
    df = pd.DataFrame(data)

    df.to_csv(path)