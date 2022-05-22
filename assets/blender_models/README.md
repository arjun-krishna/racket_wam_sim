# Blender to export .obj files

Checkbox
- X-axis forward
- Z-axis up

# Processing .obj for PyBullet

Current limitation is the materials on multi-links are not handled well need to run `extras/obj2sdf` on the obj files obtained.


**Note**: extras obj2sdf compiled from bullet3. See relevant [github issue](https://github.com/bulletphysics/bullet3/issues/1934)

```bash
cd build3
./premake4_linux64 gmake
cd gmake
make App_obj2sdf
../../bin/App_obj2sdf_* --fileName="<your>.obj"
```
