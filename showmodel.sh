export LC_NUMERIC="en_US.UTF-8" 
xacro urdf/spotmicroai.urdf.xacro > urdf/spotmicroai_gen.urdf.xml
roslaunch urdf_tutorial display.launch model:=urdf/spotmicroai_gen.urdf.xml
