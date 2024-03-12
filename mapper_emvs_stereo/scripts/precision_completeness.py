import matplotlib.pyplot as plt
import numpy as np
import pickle

plt.rcParams.update({'font.size': 14})


def precision_completeness(data, gt, name):
    print('--------- Alg: ' + name + '--------------')
    err = np.absolute(data - gt)
    print('Number of gt points: ', np.ma.count(gt))
    print('Number of estimated points: ', np.ma.count(data))
    dashed_solid = '-'
    if name == '1':
        # name = '$H_c\circ A_t$ (Alg. 1)'
        name = '$H$ (Alg. 1)'
    elif name == 'min':
        name = '${min}$'
    elif name == 'Gc':
        name = '${G}$'
    elif name == 'Ac':
        # name = '${A}_c\circ A_t$'
        name = '${A}$'
        # color = 'r'
        # dashed_solid = 'C3'
    elif name == 'rms':
        name = '${RMS}$'
    elif name == 'max':
        name = '${max}$'
    elif name == 'HcHt':
        name = '${H}_c\circ H_t$'
    elif name == '2':
        name = '$A_t\circ H_c$'
        # dashed_solid = '--'
    elif name == 'SGM':
        dashed_solid = 'C4'
    elif name == 'GTS':
        dashed_solid = 'C5'

    # precision
    ax1 = plt.figure('p')
    binwidth = 0.01
    values, base = np.histogram(err.compressed(), bins=int(np.amax(err) / binwidth))
    cumulative = np.cumsum(values) / np.ma.count(data) * 100
    plt.plot(base[:-1], cumulative, dashed_solid, label=name)
    plt.legend(prop={'size': 18}, frameon=False, loc=0)
    plt.title('Precision (%)')
    plt.xlabel('Error (m)')
    plt.xlim([0, .15])
    plt.ylim([0, None])
    # plt.ylim([0, 85])
    # plt.gca().set_aspect(0.00583)
    plt.show()
    precision = cumulative

    # completeness
    ax2 = plt.figure('c')
    binwidth = 0.01
    values, base = np.histogram(err.compressed(), bins=int(np.amax(err) / binwidth))
    cumulative = np.cumsum(values) / np.ma.count(gt) * 100
    plt.plot(base[:-1], cumulative, dashed_solid, label=name)
    plt.legend(prop={'size': 18}, frameon=False, loc=0)
    plt.title('Recall (%)')
    plt.xlabel('Error (m)')
    plt.xlim([0, .15])
    plt.ylim([0, None])
    # plt.ylim([0, 1.5])
    plt.show()
    # plt.gca().set_aspect(0.1)
    recall = cumulative

    # F1
    ax3 = plt.figure('f')
    binwidth = 0.01
    values, base = np.histogram(err.compressed(), bins=int(np.amax(err) / binwidth))
    f1 = 2 * precision * recall / (precision + recall)
    plt.plot(base[:-1], f1, dashed_solid, label=name)
    plt.legend(prop={'size': 18}, frameon=False, loc=0)
    plt.title('F1-score (%)')
    plt.xlabel('Error (m)')
    plt.xlim([0, .15])
    plt.ylim([0, None])
    # plt.ylim([0, 1.5])
    # plt.rcParams.update({'font.size': 14})
    plt.show()
    # plt.gca().set_aspect(0.1)

    # outliers
    ax1 = plt.figure('o')
    binwidth = 0.01
    values, base = np.histogram(err.compressed(), bins=int(np.amax(err) / binwidth))
    cumulative = (np.ma.count(err.compressed()) - np.cumsum(values)) / np.ma.count(err.compressed()) * 100
    plt.plot(base[:-1], cumulative, dashed_solid, label=name)
    plt.legend(prop={'size': 18}, frameon=False, loc=0)
    plt.title('% of depth points beyond error threshold')
    plt.xlabel('Error (m)')
    plt.xlim([0, None])
    plt.ylim([0, None])
    # plt.ylim([0, 85])
    # plt.gca().set_aspect(0.00583)
    plt.show()

    return ax1, ax2, ax3
