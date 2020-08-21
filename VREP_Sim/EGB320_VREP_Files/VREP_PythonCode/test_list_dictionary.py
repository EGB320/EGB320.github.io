
results = [

    {
        'rois': 2,
        'test': 0
    },
    {
        'rois': 3,
        'test': 1
    },
    {
        'rois': 3,
        'test': 1
    }
]

print(results)

rois = []

for n in range(len(results)):
    rois.append(results[n]['rois'])
    

print(rois)