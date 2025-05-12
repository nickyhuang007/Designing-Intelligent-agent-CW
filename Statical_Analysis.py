import numpy as np
from scipy.stats import ttest_ind

data = []
data_adjust = []

data.append(np.loadtxt('Output_Straight.csv', delimiter=','))
data.append(np.loadtxt('Output_Polyline.csv', delimiter=','))
data.append(np.loadtxt('Output_Quadratic.csv', delimiter=','))

data_adjust.append(np.loadtxt('adjust_fuzzy_Output_Straight.csv', delimiter=',')[:2])
data_adjust.append(np.loadtxt('adjust_fuzzy_Output_Quadratic.csv', delimiter=',')[:2])

# #Average compare
# for i in range(len(data)) :
#     print('Data',i+1)
#     data1 = data[i]

#     print('Path Length')
#     print(np.mean(data1[0]))
#     print(np.mean(data1[2]))
#     print(np.mean(data1[4]))

#     print('Run Time')
#     print(np.mean(data1[1]))
#     print(np.mean(data1[3]))
#     print(np.mean(data1[5]))

#t-test
for i in range(len(data)) :
    print('Data',i+1)
    data1 = data[i]

    print('Path Length')
    t,p = ttest_ind(data1[0],data1[4])
    print(p)
    t,p = ttest_ind(data1[2],data1[4])
    print(p)

    print('Run Time')
    t,p = ttest_ind(data1[1],data1[5])
    print(p)
    t,p = ttest_ind(data1[3],data1[5])
    print(p)

# #dajust-mean
# for i in range(len(data)-1) :
#     print('Data',i+1)
#     data1 = data_adjust[i]

#     print('Path Length')
#     print(np.mean(data1[0]))

#     print('Run Time')
#     print(np.mean(data1[1]))


# #adjust t-test
# for i in range(len(data)-1) :
#     print('Data',i+1)
#     data1 = data[i]
#     data2 = data_adjust[i]

#     print('Path Length')
#     t,p = ttest_ind(data1[0],data2[0])
#     print(p)
#     t,p = ttest_ind(data1[2],data2[0])
#     print(p)
#     t,p = ttest_ind(data1[4],data2[0])
#     print(p)


#     print('Run Time')
#     t,p = ttest_ind(data1[1],data2[1])
#     print(p)
#     t,p = ttest_ind(data1[3],data2[1])
#     print(p)
#     t,p = ttest_ind(data1[5],data2[1])
#     print(p)