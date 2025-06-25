from easydict import EasyDict as edict
import copy

def findDiff(d1, d2, path="", diff_dict = {}):
    orig_path = copy.deepcopy(path)
    for k in d1.keys():
        if not d2.has_key(k):
            print path, ":"
            print "keys not in d2: " + k, "\n"
        else:
            if type(d1[k]) in [edict, dict]:
                if path == "":
                    path = k
                else:
                    path = path + "->" + k
                diff_dict = findDiff(d1[k],d2[k], path, diff_dict)
                path = orig_path
            else:
                if d1[k] != d2[k]:
                    print path, ":"
                    print " - ", k," : ", d1[k]
                    print " + ", k," : ", d2[k] 
                    diff_dict[k] = d2[k]
                    diff_dict[k + '_dictpath'] = copy.deepcopy(path)
                    # path=""
            
    return diff_dict
        