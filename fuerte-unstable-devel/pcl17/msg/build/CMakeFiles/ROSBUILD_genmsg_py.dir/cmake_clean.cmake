FILE(REMOVE_RECURSE
  "../../msg_gen"
  "../../src/pcl17/msg"
  "../../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../../src/pcl17/msg/__init__.py"
  "../../src/pcl17/msg/_Vertices.py"
  "../../src/pcl17/msg/_PointIndices.py"
  "../../src/pcl17/msg/_ModelCoefficients.py"
  "../../src/pcl17/msg/_PolygonMesh.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
