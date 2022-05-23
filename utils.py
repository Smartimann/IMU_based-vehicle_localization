import numpy as np 
import pandas as pd

from lxml import etree
import math
#from __future__ import print_function

def write_to_csv(name, data): 
    path = 'data/'+str(name)+'.csv'
    df = pd.DataFrame(data)

    df.to_csv(path)



