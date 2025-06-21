function path_following(goal)
global changdu
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
if (clientID<0)
    disp('Failed connecting to remote API server');
else
    [~,robot]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
    [~,robotpos]=vrep.simxGetObjectPosition(clientID,robot,-1,vrep.simx_opmode_blocking);
    [~,attitude]=vrep.simxGetObjectOrientation(clientID,robot,-1,vrep.simx_opmode_oneshot_wait);
    [~,rmotor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    [~,lmotor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    stage=0;
    opos=robotpos;
    while stage==0
            deltax=goal(1)-robotpos(1);%x error
            deltay=goal(2)-robotpos(2);%y error
            phi=angle(deltax+deltay*1i);
            deltaangle=-(attitude(3)-phi);%angle error
            dis=distance(robotpos(1),robotpos(2),goal(1),goal(2));%distance computing
            vdes=0.1;
            omdes=0.4*deltaangle;
            [o_l,o_r]=move(vdes,omdes);%computing w1 and w2 based on the value v and w
            vrep.simxSetJointTargetVelocity(clientID,lmotor,o_l,vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,rmotor,o_r,vrep.simx_opmode_oneshot);
            
            [~,attitude]=vrep.simxGetObjectOrientation(clientID,robot,-1,vrep.simx_opmode_oneshot_wait);
            [~,robotpos]=vrep.simxGetObjectPosition(clientID,robot,-1,vrep.simx_opmode_blocking);
            changdu=changdu+distance(opos(1),opos(2),robotpos(1),robotpos(2));
            opos=robotpos;
            dis=distance(robotpos(1),robotpos(2),goal(1),goal(2));
        if dis<0.1
            stage=1;
            [o_l,o_r]=move(0,0);
            vrep.simxSetJointTargetVelocity(clientID,lmotor,o_l,vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,rmotor,o_r,vrep.simx_opmode_oneshot);
        end
    end
end
            