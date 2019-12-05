import os
import trimesh
import numpy as np
from matplotlib import pyplot as plt

path = os.getcwd() + '/kit/'
file_list = os.listdir(path)
name_list = []
for file_name in file_list:
    name = file_name.replace(".obj","")
    trimesh_data = trimesh.load(path + name + '.obj')
    if np.min(trimesh_data.bounds[1] - trimesh_data.bounds[0]) > 0.03 and np.max(trimesh_data.bounds[1] - trimesh_data.bounds[0]) > 0.10:
        name_list.append(name)
        newpath = os.getcwd() + '/.gazebo/models/'
        try:
            os.mkdir(newpath + name)
            os.mkdir(newpath + name + '/meshes')
        except:
            print(name + " already exsit")

        unit = 1.0
        if unit**3*trimesh_data.mass < 1e-5:
            mass = 1e-5
        else:
            mass = unit**3*trimesh_data.mass
        ratio = mass / np.abs(unit**3*trimesh_data.mass)
        inertia_sign = np.sign(trimesh_data.moment_inertia[0][0])
        if "DanishHam" in name:
            print("Volume is "+str(trimesh_data.mass))
            print("CoM is "+str(trimesh_data.center_mass[0])+" "+str(trimesh_data.center_mass[1])+" "+str(trimesh_data.center_mass[2]))
            print("Inertia is "+str(trimesh_data.moment_inertia))
        sdf_content = "<?xml version=\"1.0\" ?>\n"
        sdf_content += "<sdf version=\"1.5\">\n"
        sdf_content += "<model name=\""+name+"\">\n"
        sdf_content += "  <link name=\"link\">\n"
        sdf_content += "    <inertial>\n"
        sdf_content += "      <pose>"+str(unit*trimesh_data.center_mass[0])+" "+str(unit*trimesh_data.center_mass[1])+" "+str(unit*trimesh_data.center_mass[2])+" 0 0 0</pose>\n"
        sdf_content += "      <mass>"+str(mass)+"</mass>\n"
        sdf_content += "      <inertia>\n"
        sdf_content += "        <ixx>"+str(unit**2*ratio*inertia_sign*trimesh_data.moment_inertia[0][0])+"</ixx>\n"
        sdf_content += "        <ixy>"+str(unit**2*ratio*inertia_sign*trimesh_data.moment_inertia[0][1])+"</ixy>\n"
        sdf_content += "        <ixz>"+str(unit**2*ratio*inertia_sign*trimesh_data.moment_inertia[0][2])+"</ixz>\n"
        sdf_content += "        <iyy>"+str(unit**2*ratio*inertia_sign*trimesh_data.moment_inertia[1][1])+"</iyy>\n"
        sdf_content += "        <iyz>"+str(unit**2*ratio*inertia_sign*trimesh_data.moment_inertia[1][0])+"</iyz>\n"
        sdf_content += "        <izz>"+str(unit**2*ratio*inertia_sign*trimesh_data.moment_inertia[2][2])+"</izz>\n"
        sdf_content += "      </inertia>\n"
        sdf_content += "    </inertial>\n"
        sdf_content += "    <collision name=\"collision\">\n"
        sdf_content += "      <max_contacts>6</max_contacts>\n"
        sdf_content += "      <geometry>\n"
        sdf_content += "        <mesh><uri>model://"+name+"/meshes/"+name+".dae</uri><scale>1.0 1.0 1.0</scale></mesh>\n"
        sdf_content += "      </geometry>\n"
        sdf_content += "      <surface>\n"
        sdf_content += "        <friction>\n"
        sdf_content += "          <!--<torsional>\n"
        sdf_content += "            <patch_radius>0.15</patch_radius>\n"
        sdf_content += "            <ode>\n"
        sdf_content += "              <slip>1.0</slip>\n"
        sdf_content += "            </ode>\n"
        sdf_content += "          </torsional>-->\n"
        sdf_content += "          <ode>\n"
        sdf_content += "            <mu>1.</mu>\n"
        sdf_content += "            <mu2>1.</mu2>\n"
        sdf_content += "            <slip1>1.</slip1>\n"
        sdf_content += "            <slip2>1.</slip2>\n"
        sdf_content += "          </ode>\n"
        sdf_content += "        </friction>\n"
        sdf_content += "        <contact>\n"
        sdf_content += "          <ode>\n"
        sdf_content += "            <soft_cfm>0.8</soft_cfm>\n"
        sdf_content += "            <soft_erp>0.2</soft_erp>\n"
        sdf_content += "            <kp>10000.0</kp>\n"
        sdf_content += "            <kd>90.0</kd>\n"
        sdf_content += "            <max_vel>0.0</max_vel>\n"
        sdf_content += "            <min_depth>0.0</min_depth>\n"
        sdf_content += "          </ode>\n"
        sdf_content += "        </contact>\n"
        sdf_content += "      </surface>\n"
        sdf_content += "    </collision>\n"
        sdf_content += "    <visual name=\"visual\">\n"
        sdf_content += "      <geometry>\n"
        sdf_content += "        <mesh><uri>model://"+name+"/meshes/"+name+".dae</uri><scale>1.0 1.0 1.0</scale></mesh>\n"
        sdf_content += "      </geometry>\n"
        sdf_content += "      <material>\n"
        sdf_content += "        <ambient>5. .0 .3 1</ambient>\n"
        sdf_content += "        <diffuse>5. .0 .3 1</diffuse>\n"
        sdf_content += "        <specular>.2 .2 .2 64</specular>\n"
        sdf_content += "        <emissive>.1 0 .1 1</emissive>\n"
        sdf_content += "      </material>\n"
        sdf_content += "    </visual>\n"
        sdf_content += "    <sensor name=\""+name+"_contact\" type=\"contact\">\n"
        sdf_content += "      <always_on>true</always_on>\n"
        sdf_content += "      <contact>\n"
        sdf_content += "        <collision>collision</collision>\n"
        sdf_content += "      </contact>\n"
        sdf_content += "      <plugin name=\""+name+"_gazebo_ros_bumper_controller\" filename=\"libgazebo_ros_bumper.so\">\n"
        sdf_content += "        <bumperTopicName>/gazebo/"+name+"/contact</bumperTopicName>\n"
        sdf_content += "        <frameName>world</frameName>\n"
        sdf_content += "      </plugin>\n"
        sdf_content += "    </sensor>\n"
        sdf_content += "  </link>\n"
        sdf_content += "</model>\n"
        sdf_content += "</sdf>"
        config_content = "<?xml version=\"1.0\"?>\n"
        config_content += "<model>\n"
        config_content += "  <name>"+name+"</name>\n"
        config_content += "  <version>1.0</version>\n"
        config_content += "  <sdf version=\"1.5\">model.sdf</sdf>\n"
        config_content += "  <author>\n"
        config_content += "    <name>Lee</name>\n"
        config_content += "  </author>\n"
        config_content += "  <description>\n"
        config_content += "    "+name+"\n"
        config_content += "  </description>\n"
        config_content += "</model>"

        trimesh.exchange.export.export_mesh(trimesh_data, newpath + name+'/meshes/'+name+'.stl', file_type="stl")
        with open(newpath + name+'/model.sdf','w') as f:
            f.write(sdf_content)
        f.close()
        with open(newpath + name+'/model.config','w') as f:
            f.write(config_content)
        f.close()
