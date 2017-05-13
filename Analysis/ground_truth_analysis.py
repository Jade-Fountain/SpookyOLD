import json

file = open('example_FK_data.fkmcp')

json_string = file.read()

json_data = json.loads(json_string)

for i in range(len(json_data)):
	print "Data ", i, ":"
	print json_data[i]

# print json_data

file.close()