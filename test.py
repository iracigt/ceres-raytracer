import ceres_rt
from PIL import Image 

color = ceres_rt.Vector3(0.9, 0.9, 0.9)

cube_pos = ceres_rt.Vector3(0,0,0)
cam_pos = ceres_rt.Vector3(0,0,3)
light_pos = ceres_rt.Vector3(0,0,4)
light = ceres_rt.PointLight(light_pos, 1.0)
cam = ceres_rt.PinholeCamera(30, (1024.0, 1024.0), (20.0, 20.0), cam_pos)
obj = ceres_rt.Entity(color, "../../Bennu_v20_200k.obj")

scene = ceres_rt.Scene()
scene.add_entity(obj, 2, cube_pos)
scene.add_light(light)

arr = scene.render(cam)
print(arr.shape)
Image.fromarray(arr).save('out.png')