import numpy as np

class TreeNode:
    def __init__(self, num, orient):
        self.children = []
        self.orientation = orient
        self.location = num

def buildTree(node,nodeMap,stopCon):
    if not stopCon:
        orient = node.orientation
        loc = node.location
        neigh, c, newOr = findNeighbors(loc,nodeMap,orient)
        
        if 13 in neigh:
            stopCon = True

        for i in range(len(neigh)):
            child = TreeNode(neigh[i],newOr[i])

            currChild = buildTree(child,nodeMap,stopCon)
            node.children.append(currChild)
    #end loop

    return node


def findNeighbors(nodeStart,nodeMap,orient):

    #NO U-TURNS!
    newOr = []
    neighbors = []
    stepCosts = []

    loc = nodeMap[nodeStart]['coord']
    loc = np.array(loc)

    stepSearch = [1,1,-1,-1]
    halfSearch = [0.5,0.5,-0.5,-0.5]

    allowDir = nodeMap[nodeStart]['allow']
    endAllow = nodeMap[13]['allow']

    if orient == 1:
        allowDir[2] = 0
    elif orient == 2:
        allowDir[3] = 0
    elif orient == 3:
        allowDir[0] = 0
    elif orient == 4:
        allowDir[1] = 0

    for i in range(4):
        if allowDir[i] == 1:
            if i%2:
                searchLoc = loc + np.array([0, stepSearch[i]])
                searchLocH = loc + np.array([0, halfSearch[i]])
            else:
                searchLoc = loc + np.array([stepSearch[i],0])
                searchLocH = loc + np.array([halfSearch[i],0])

            nodeN = searchCoords(nodeMap,searchLoc)
            HnodeN = searchCoords(nodeMap,searchLocH)
            if HnodeN == 12:
                HnodeN = None

            if HnodeN == 13:
                if not endAllow[i]:
                    HnodeN = None

            if HnodeN:
                neighbors.append(HnodeN)
                stepCosts.append(0.5)
                newOr.append(i+1)
            elif not nodeN == None:
                neighbors.append(nodeN)
                stepCosts.append(1)
                newOr.append(i+1)

    return neighbors, stepCosts, newOr


def searchCoords(nodeMap, loc):
    for i, dic in enumerate(nodeMap):
        if dic['coord'][0] == loc[0] and dic['coord'][1] == loc[1]:
            return i
    return None


def buildMap(sblock,spos,eblock,epos):
    nodes = []
    for i in range(14):
        nodes.append({'coord':[],'allow':[],'manhat':[]})

    
    nodes[0]['coord'] = [2,0]
    nodes[0]['allow'] = [0,1,0,0]

    nodes[1]['coord'] = [1,0]
    nodes[1]['allow'] = [1,1,0,0]

    nodes[2]['coord'] = [0,0]
    nodes[2]['allow'] = [1,0,0,0]

    nodes[3]['coord'] = [2,1]
    nodes[3]['allow'] = [0,1,1,0]

    nodes[4]['coord'] = [1,1]
    nodes[4]['allow'] = [0,1,1,1]

    nodes[5]['coord'] = [0,1]
    nodes[5]['allow'] = [0,0,0,1]

    nodes[6]['coord'] = [2,2]
    nodes[6]['allow'] = [0,1,1,0]

    nodes[7]['coord'] = [1,2]
    nodes[7]['allow'] = [1,1,1,1]

    nodes[8]['coord'] = [0,2]
    nodes[8]['allow'] = [1,0,0,1]

    nodes[9]['coord'] = [2,3]
    nodes[9]['allow'] = [0,0,1,0]

    nodes[10]['coord'] = [1,3]
    nodes[10]['allow'] = [0,0,1,1]

    nodes[11]['coord'] = [0,3]
    nodes[11]['allow'] = [0,0,0,1]

    if spos == 1:
        orient = 2
    elif spos == 2:
        orient = 3
    elif spos == 3:
        orient = 4
    elif spos == 4:
        orient = 1

    if sblock == 2 or sblock == 5:
        if spos == 4:
            orient = 3

    nodes[12] = buildNode(sblock,spos)
    nodes[13] = buildNode(eblock,epos)

    for i in range(len(nodes)):
        sx = nodes[i]['coord'][0]
        sy = nodes[i]['coord'][1]
        ex = nodes[13]['coord'][0]
        ey = nodes[13]['coord'][1]

        nodes[i]['manhat'] = abs(sx-ex) + abs(sy-ey)
    return nodes, orient

def buildNode(block,pos):
    node = {'coord':[],'allow':[]}
    if block == 1:
        node['coord'] = np.array([1.5,0.5])
    elif block == 2:
        node['coord'] = np.array([1.5,1.5])
    elif block == 3:
        node['coord'] = np.array([1.5,2.5])
    elif block == 4:
        node['coord'] = np.array([0.5,0.5])
    elif block == 5:
        node['coord'] = np.array( [0.5,1.5])
    elif block == 6:
        node['coord'] = np.array([0.5,2.5])

    if pos == 1:
        node['coord'] = node['coord'] + np.array([0.5,0])
        node['allow'] = [0,1,0,0]
    elif pos == 2:
        node['coord'] = node['coord'] + np.array([0,0.5])
        node['allow'] = [0,0,1,0]
    elif pos == 3:
        node['coord'] = node['coord'] + np.array([-0.5,0])
        node['allow'] = [0,0,0,1]
    elif pos == 4:
        node['coord'] = node['coord'] + np.array([0,-0.5])
        node['allow'] = [1,0,0,0]

    return node


# Example Map
cityMap = buildMap(6,3,2,1)
cityTree = TreeNode(12,4)
cityTree = buildTree(cityTree,cityMap,False)
