import yaml

def get_package_config (filename):
    '''
    Gets the task requirement information

    Args:
        filename (str): configuration filename. Usually `config.yaml`, provided by eYRC
    Returns:
        rack info dictionary. Refer rack_pose_info assignment under __name__ == "__main__" for details
    '''
    config_f = open ('config.yaml')
    rack_info = yaml.safe_load (config_f)
    config_f.close ()

    drop_f = open ('poses.yaml')
    drop_poses = yaml.safe_load (drop_f)
    drop_f.close ()

    rack_poses = []
    box_ids = rack_info['package_id']
    for box in box_ids:
        for rack in rack_info['position']:
            if f'rack{box}' in rack:
                rack = rack[f'rack{box}']
                rack_poses += [{
                    'pickup': {'trans': [rack[0], rack[1]], 'rot': rack[2]},
                    #'drop': {'trans': [0.85, -2.455], 'rot': 3.14},
                    'drop': drop_poses.pop (),
                    'rack': f'rack{box}',
                    'box': box,
                }]
                break

    return rack_poses