import yaml
from math import pi

def get_package_config (filename):
    '''
    Gets the task requirement information

    Args:
        filename (str): configuration filename. Usually `config.yaml`, provided by eYRC
    Returns:
        rack info dictionary. Refer rack_pose_info assignment under __name__ == "__main__" in task2b.py for details
    '''

    arm_pos = [1.6, -2.45]

    config_f = open (filename)
    config = yaml.safe_load(config_f)
    config_f.close ()
    
    box_ids = config['package_id']
    rack_info = []
    for rack in config['position']:
        rack = list(rack.items ())[0]
        _id = rack[0].split("rack")[1]
        if not int(_id) in box_ids: continue
        trans = rack[1][0:2]
        rot = rack[1][2]
        rack_info += [{
            'pickup': { 'trans': trans, 'rot': rot },
            'rack': f'rack{_id}',
            'box': int(_id)
        }]
    rack_info.sort (key = lambda rack:
        (rack['pickup']['trans'][0] - arm_pos[0])**2 + (rack['pickup']['trans'][1] - arm_pos[1])**2
    )

    approach_angles = {1: [0.0, 0.8], 2: [-0.8, 0.0], 3: [0.0, -0.8]} # n: [x, y] => approach at n*pi/2, reach offset (x,y) from arm
    for rack in rack_info:
        x, y = rack['pickup']['trans']
        x -= arm_pos[0]; y -= arm_pos[1]

        # ap is angle at which rack approaches arm, represented as a multiple of pi/2
        ap = abs(y) > abs(x)  # points along +y if y component is greater
        if x + y < 0: ap += 2 # reversed if the larger component is less than 0

        if ap not in approach_angles:
            drop = None
        else:
            drop = (ap, approach_angles.pop (ap)) # Approach angle, final position
        rack['drop'] = drop


    for rack in rack_info:
        ap, pos = rack['drop'] or approach_angles.popitem ()
        pos[0] += arm_pos[0]; pos[1] += arm_pos[1]
        rack['drop'] = {'trans': pos, 'rot': ap * pi / 2}

    return rack_info


if __name__ == "__main__":
    for i in get_package_config ("../../../config.yaml"):
        print (i)