# command test


outcommands = ['L','S','R','S','S','R']
i = -1
newcomm = []
while i >= -1*len(outcommands):
	if outcommands[i] != 'S':
		newcomm.append(outcommands[i])
		i = i-1
	else:
		goS = 0
		while outcommands[i-1] == 'S':
			goS = goS + 1
			i = i-1
		i = i-1
		newcomm.append('S{}'.format(goS))
print(newcomm)