from stl2obj import Stl2Obj

callback = lambda code: print(code)

src = 'urdf/PAI-urdf/meshes/base_link.STL'  # may be stl or obj
dst = 'urdf/PAI-urdf/meshes/base_link.obj'  # may be stl or obj
Stl2Obj().convert(src, dst, callback)

src = 'urdf/PAI-urdf/meshes/left-ankle.STL'  # may be stl or obj
dst = 'urdf/PAI-urdf/meshes/left-ankle.obj'  # may be stl or obj
Stl2Obj().convert(src, dst, callback)
src = 'urdf/PAI-urdf/meshes/left-hip-abad.STL'  # may be stl or obj
dst = 'urdf/PAI-urdf/meshes/left-hip-abad.obj'  # may be stl or obj
Stl2Obj().convert(src, dst, callback)
src = 'urdf/PAI-urdf/meshes/left-hip-pitch.STL'  # may be stl or obj
dst = 'urdf/PAI-urdf/meshes/left-hip-pitch.obj'  # may be stl or obj
Stl2Obj().convert(src, dst, callback)
src = 'urdf/PAI-urdf/meshes/left-hip-yaw.STL'  # may be stl or obj
dst = 'urdf/PAI-urdf/meshes/left-hip-yaw.obj'  # may be stl or obj
Stl2Obj().convert(src, dst, callback)
src = 'urdf/PAI-urdf/meshes/left-knee.STL'  # may be stl or obj
dst = 'urdf/PAI-urdf/meshes/left-knee.obj'  # may be stl or obj
Stl2Obj().convert(src, dst, callback)

###################################################################################

src = 'urdf/PAI-urdf/meshes/right-ankle.STL'  # may be stl or obj
dst = 'urdf/PAI-urdf/meshes/right-ankle.obj'  # may be stl or obj
Stl2Obj().convert(src, dst, callback)
src = 'urdf/PAI-urdf/meshes/right-hip-abad.STL'  # may be stl or obj
dst = 'urdf/PAI-urdf/meshes/right-hip-abad.obj'  # may be stl or obj
Stl2Obj().convert(src, dst, callback)
src = 'urdf/PAI-urdf/meshes/right-hip-pitch.STL'  # may be stl or obj
dst = 'urdf/PAI-urdf/meshes/right-hip-pitch.obj'  # may be stl or obj
Stl2Obj().convert(src, dst, callback)
src = 'urdf/PAI-urdf/meshes/right-hip-yaw.STL'  # may be stl or obj
dst = 'urdf/PAI-urdf/meshes/right-hip-yaw.obj'  # may be stl or obj
Stl2Obj().convert(src, dst, callback)
src = 'urdf/PAI-urdf/meshes/right-knee.STL'  # may be stl or obj
dst = 'urdf/PAI-urdf/meshes/right-knee.obj'  # may be stl or obj
Stl2Obj().convert(src, dst, callback)
