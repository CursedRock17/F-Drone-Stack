# Basic Examples 
-------------------------

### Running the World
1) Open two instances of the terminal (in the hardware section)
2) In the first terminal run `gz sim worlds/blank_world.sdf` to open the empty world
3) In the second terminal
   `gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "models/f_drone.urdf", name: "f_drone"'`
