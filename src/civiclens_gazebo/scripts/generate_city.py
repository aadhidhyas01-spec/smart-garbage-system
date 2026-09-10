import os

def generate_world():
    sdf = []
    sdf.append('<?xml version="1.0" ?>')
    sdf.append('<sdf version="1.6">')
    sdf.append('  <world name="civiclens_indian_city">')
    sdf.append('    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>')
    sdf.append('    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>')
    
    # Lighting (Warm Sunny Day)
    sdf.append('''
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 80 0 0 0</pose>
      <diffuse>0.98 0.95 0.88 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.4 0.3 -0.85</direction>
    </light>
    ''')

    # Ground (Asphalt Road Surface)
    sdf.append('''
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>350 350</size></plane></geometry>
        </collision>
        <visual name="asphalt">
          <geometry><plane><normal>0 0 1</normal><size>350 350</size></plane></geometry>
          <material>
            <ambient>0.22 0.22 0.24 1</ambient>
            <diffuse>0.25 0.25 0.27 1</diffuse>
            <specular>0.08 0.08 0.08 1</specular>
          </material>
        </visual>
      </link>
    </model>
    ''')

    # Base Station
    sdf.append('''
    <include>
      <name>base_station</name>
      <uri>model://base_station</uri>
      <pose>0 -10 0 0 0 0</pose>
    </include>
    ''')

    # Truck starting position at Base Station
    sdf.append('''
    <include>
      <name>civiclens_truck</name>
      <uri>model://civiclens_truck</uri>
      <pose>0 -10 0.2 0 0 1.57</pose>
    </include>
    ''')

    # Generate 4x4 City Blocks
    # Road width = 10 (from -5 to 5 around multiples of 50: 0, 50, 100, 150, 200)
    # Block size = 40x40 (from 5 to 45)
    # Block centers: bx = col * 50 + 25, by = row * 50 + 25
    house_id = 1
    tree_id = 1

    for row in range(4):
        for col in range(4):
            bx = col * 50 + 25
            by = row * 50 + 25

            # Paved Sidewalk Base for each block
            sdf.append(f'''
            <model name="sidewalk_block_{row}_{col}">
              <static>true</static>
              <pose>{bx} {by} 0.04 0 0 0</pose>
              <link name="link">
                <collision name="col">
                  <geometry><box><size>40 40 0.08</size></box></geometry>
                </collision>
                <visual name="pavement">
                  <geometry><box><size>40 40 0.08</size></box></geometry>
                  <material>
                    <ambient>0.72 0.70 0.65 1</ambient>
                    <diffuse>0.78 0.75 0.70 1</diffuse>
                    <specular>0.1 0.1 0.1 1</specular>
                  </material>
                </visual>
              </link>
            </model>
            ''')

            # The central public park block (row 2, col 2)
            if row == 2 and col == 2:
                # Park Grass Surface
                sdf.append(f'''
                <model name="park_{row}_{col}">
                  <static>true</static>
                  <pose>{bx} {by} 0.09 0 0 0</pose>
                  <link name="link">
                    <visual name="grass">
                      <geometry><box><size>38 38 0.02</size></box></geometry>
                      <material>
                        <ambient>0.20 0.58 0.18 1</ambient>
                        <diffuse>0.25 0.68 0.22 1</diffuse>
                      </material>
                    </visual>
                    <!-- Park Walking Pathway -->
                    <visual name="path_horizontal">
                      <geometry><box><size>38 3.0 0.03</size></box></geometry>
                      <material><ambient>0.65 0.60 0.52 1</ambient></material>
                    </visual>
                    <visual name="path_vertical">
                      <geometry><box><size>3.0 38 0.03</size></box></geometry>
                      <material><ambient>0.65 0.60 0.52 1</ambient></material>
                    </visual>
                    <!-- Central Gazebo Pavilion -->
                    <visual name="central_fountain_base">
                      <geometry><cylinder><radius>3.5</radius><length>0.4</length></cylinder></geometry>
                      <material><ambient>0.85 0.85 0.85 1</ambient></material>
                    </visual>
                    <visual name="central_fountain_water">
                      <pose>0 0 0.25 0 0 0</pose>
                      <geometry><cylinder><radius>2.8</radius><length>0.15</length></cylinder></geometry>
                      <material>
                        <ambient>0.15 0.45 0.65 0.8</ambient>
                        <diffuse>0.2 0.55 0.75 0.8</diffuse>
                      </material>
                    </visual>
                  </link>
                </model>
                ''')
                # Park Trees
                for tx in [-12, -4, 4, 12]:
                    for ty in [-12, 12]:
                        sdf.append(f'''
                        <model name="park_tree_{tree_id}">
                          <static>true</static>
                          <pose>{bx+tx} {by+ty} 0.08 0 0 0</pose>
                          <link name="link">
                            <visual name="trunk">
                              <pose>0 0 1.2 0 0 0</pose>
                              <geometry><cylinder><radius>0.22</radius><length>2.4</length></cylinder></geometry>
                              <material><ambient>0.35 0.20 0.08 1</ambient></material>
                            </visual>
                            <visual name="canopy">
                              <pose>0 0 3.0 0 0 0</pose>
                              <geometry><sphere><radius>1.8</radius></sphere></geometry>
                              <material>
                                <ambient>0.15 0.52 0.15 1</ambient>
                                <diffuse>0.20 0.60 0.20 1</diffuse>
                              </material>
                            </visual>
                          </link>
                        </model>
                        ''')
                        tree_id += 1
            else:
                # Residential Houses in Blocks!
                # Each block has houses facing the roads:
                # Southern row of houses in block (facing South towards road at by - 25)
                for hx in [-10, 10]:
                    sdf.append(f'''
                    <include>
                      <name>house_{house_id}</name>
                      <uri>model://house</uri>
                      <pose>{bx+hx} {by-10} 0.08 0 0 0</pose>
                    </include>
                    ''')
                    house_id += 1

                # Northern row of houses in block (facing North towards road at by + 25)
                for hx in [-10, 10]:
                    sdf.append(f'''
                    <include>
                      <name>house_{house_id}</name>
                      <uri>model://house</uri>
                      <pose>{bx+hx} {by+10} 0.08 0 0 3.14159</pose>
                    </include>
                    ''')
                    house_id += 1

                # Roadside Avenue Tree between houses
                sdf.append(f'''
                <model name="street_tree_{tree_id}">
                  <static>true</static>
                  <pose>{bx} {by-18} 0.08 0 0 0</pose>
                  <link name="link">
                    <visual name="trunk">
                      <pose>0 0 1.2 0 0 0</pose>
                      <geometry><cylinder><radius>0.18</radius><length>2.4</length></cylinder></geometry>
                      <material><ambient>0.35 0.20 0.08 1</ambient></material>
                    </visual>
                    <visual name="canopy">
                      <pose>0 0 2.8 0 0 0</pose>
                      <geometry><sphere><radius>1.5</radius></sphere></geometry>
                      <material>
                        <ambient>0.18 0.55 0.18 1</ambient>
                        <diffuse>0.22 0.62 0.22 1</diffuse>
                      </material>
                    </visual>
                  </link>
                </model>
                ''')
                tree_id += 1

    # 5 Smart Dustbins & Solar AI Camera IoT Stations for 5 Streets
    # Street 1: Greenfield Road (Biodegradable)
    # Street 2: Market Lane (Non-Biodegradable)
    # Street 3: Heritage Avenue (Plastic)
    # Street 4: Sunrise Boulevard (Biodegradable)
    # Street 5: Tech Park Way (Plastic)

    streets = [
        {"id": 1, "name": "STREET 1: GREENFIELD RD", "bx": 30.0, "by": -3.5, "yaw": 1.5707, "sign_x": 26.0, "sign_y": -3.5},
        {"id": 2, "name": "STREET 2: MARKET LANE", "bx": 53.5, "by": 30.0, "yaw": 3.14159, "sign_x": 53.5, "sign_y": 26.0},
        {"id": 3, "name": "STREET 3: HERITAGE AVE", "bx": 30.0, "by": 53.5, "yaw": -1.5707, "sign_x": 26.0, "sign_y": 53.5},
        {"id": 4, "name": "STREET 4: SUNRISE BLVD", "bx": -3.5, "by": 30.0, "yaw": 0.0, "sign_x": -3.5, "sign_y": 26.0},
        {"id": 5, "name": "STREET 5: TECH PARK WAY", "bx": 80.0, "by": -3.5, "yaw": 1.5707, "sign_x": 76.0, "sign_y": -3.5}
    ]

    for st in streets:
        sid = st["id"]
        bx, by, yaw = st["bx"], st["by"], st["yaw"]
        # Smart Dustbin
        sdf.append(f'<include><name>dustbin_{sid}</name><uri>model://dustbin</uri><pose>{bx} {by} 0 0 0 {yaw}</pose></include>')
        # Solar-powered AI Camera Station
        cx = bx + (1.5 if abs(yaw) < 2.0 and yaw != 0.0 else 0.0)
        cy = by + (1.5 if yaw == 0.0 or abs(yaw) > 3.0 else 0.0)
        sdf.append(f'<include><name>camera_station_{sid}</name><uri>model://ai_camera</uri><pose>{cx} {cy} 0 0 0 0</pose></include>')
        # Waste Entity
        sdf.append(f'<include><name>waste_{sid}</name><uri>model://waste</uri><pose>{bx} {by} 0.76 0 0 0</pose></include>')
        
        # Street Signpost Marker
        sx, sy = st["sign_x"], st["sign_y"]
        sdf.append(f'''
        <model name="street_sign_{sid}">
          <static>true</static>
          <pose>{sx} {sy} 0 0 0 0</pose>
          <link name="link">
            <visual name="post">
              <pose>0 0 1.2 0 0 0</pose>
              <geometry><cylinder><radius>0.04</radius><length>2.4</length></cylinder></geometry>
              <material><ambient>0.3 0.3 0.35 1</ambient></material>
            </visual>
            <visual name="sign_board">
              <pose>0 0 2.2 0 0 0</pose>
              <geometry><box><size>1.6 0.06 0.4</size></box></geometry>
              <material>
                <ambient>0.12 0.35 0.65 1</ambient>
                <diffuse>0.15 0.45 0.85 1</diffuse>
              </material>
            </visual>
            <visual name="sign_border">
              <pose>0 0 2.2 0 0 0</pose>
              <geometry><box><size>1.64 0.05 0.44</size></box></geometry>
              <material><ambient>0.9 0.9 0.9 1</ambient></material>
            </visual>
          </link>
        </model>
        ''')

    # Base Station Recycling & Shredder Facility Marker
    sdf.append('''
    <model name="base_shredder_plant">
      <static>true</static>
      <pose>4.5 -12.0 0 0 0 0</pose>
      <link name="link">
        <visual name="shredder_chassis">
          <pose>0 0 1.0 0 0 0</pose>
          <geometry><box><size>2.2 2.0 2.0</size></box></geometry>
          <material>
            <ambient>0.2 0.22 0.25 1</ambient>
            <diffuse>0.25 0.28 0.32 1</diffuse>
          </material>
        </visual>
        <visual name="shredder_hopper">
          <pose>0 0 2.2 0 0 0</pose>
          <geometry><cylinder><radius>0.6</radius><length>0.6</length></cylinder></geometry>
          <material>
            <ambient>0.85 0.65 0.1 1</ambient>
            <diffuse>0.92 0.72 0.12 1</diffuse>
          </material>
        </visual>
        <visual name="recycling_sign">
          <pose>0 -1.02 1.4 0 0 0</pose>
          <geometry><box><size>1.8 0.04 0.5</size></box></geometry>
          <material>
            <ambient>0.15 0.6 0.2 1</ambient>
            <diffuse>0.18 0.7 0.25 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    ''')

    sdf.append('''
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
''')

    target_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'worlds', 'civiclens_indian_city.world'))
    with open(target_path, 'w') as f:
        f.write('\n'.join(sdf))
    print(f"World successfully generated at: {target_path} (Total Houses: {house_id - 1})")

if __name__ == '__main__':
    generate_world()

