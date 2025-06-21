function [o_l,o_r]=move(v,omega)%computing w1 and w2 based on the value v and w
wheel_radius=0.0975;
distance=0.415;
v_l=v-omega*distance;
v_r=v+omega*distance;
o_l=v_l/wheel_radius;
o_r=v_r/wheel_radius;
