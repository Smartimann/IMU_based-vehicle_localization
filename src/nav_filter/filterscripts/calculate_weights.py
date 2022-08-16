from sklearn import preprocessing
def calculate_weights_from_differences(differences): 
    return 1-preprocessing.minmax_scale(differences)

def get_weight_mean(rot_weights, acc_weights, dm_weights): 
    return (rot_weights + acc_weights + dm_weights)/3