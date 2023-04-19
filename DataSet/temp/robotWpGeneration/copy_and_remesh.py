import pymeshlab
import glob, os
import shutil
import numpy as np

#copy files into CURVED_LAYER and TOOL_PATH
source_path = os.path.abspath(os.path.join(os.getcwd(), "../.."))
source_path_layer = source_path + "\CURVED_LAYER"
source_path_toolpath = source_path + "\TOOL_PATH"
#print(source_path_layer)
#print(source_path_toolpath)

target_path = os.getcwd();
target_path_layer = target_path + "\CURVED_LAYER"
target_path_toolpath = target_path + "\TOOL_PATH"
#print(target_path_layer)
#print(target_path_toolpath)
#os.system("pause")

if not os.path.exists(target_path_layer):
    os.makedirs(target_path_layer)

if os.path.exists(source_path_layer):
    shutil.rmtree(target_path_layer)

shutil.copytree(source_path_layer, target_path_layer)
print('copy CURVED_LAYER finished!')

if not os.path.exists(target_path_toolpath):
    os.makedirs(target_path_toolpath)

if os.path.exists(source_path_toolpath):
    shutil.rmtree(target_path_toolpath)

shutil.copytree(source_path_toolpath, target_path_toolpath)
print('copy TOOL_PATH finished!')
#os.system("pause")

# remesh
rootDir = os.getcwd()
print(rootDir)
#os.system("pause")

files = glob.glob(rootDir + "/layer_Simplified/*")
for f in files:
    os.remove(f)

os.chdir(rootDir + "/CURVED_LAYER")
for file in glob.glob("*.obj"):
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(file)
    ms.load_filter_script(rootDir + "/remesh_operation.mlx")
    ms.apply_filter_script()
    ms.save_current_mesh(rootDir + "/layer_Simplified/" + file)
    print("remesh " , file, " finished.")
