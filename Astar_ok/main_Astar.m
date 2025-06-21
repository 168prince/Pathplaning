clear;clc;
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
if (clientID<0)
    disp('Failed connecting to remote API server');
else
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
            %�����˾��
        [~,robot]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
        [~,rmotor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
        [~,lmotor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    %����յ���   
        [~,realpos]=vrep.simxGetObjectHandle(clientID,'realpos',vrep.simx_opmode_blocking);
        [~,desirepos]=vrep.simxGetObjectHandle(clientID,'desirepos',vrep.simx_opmode_blocking);
       %����յ�����        
        [~,realpospos]=vrep.simxGetObjectPosition(clientID,realpos,-1,vrep.simx_opmode_blocking);
        [~,desirepospos]=vrep.simxGetObjectPosition(clientID,desirepos,-1,vrep.simx_opmode_blocking);
    %��������
        [~,robotpos]=vrep.simxGetObjectPosition(clientID,robot,-1,vrep.simx_opmode_blocking);
        finalvpath=[];
        while true
        %% ·���滮
        tic;
        path=Astar;
        vpath=(path-5.5)/2;
        vpath(:,3)=0.05;
        for i=2:size(vpath,1)
            if norm(vpath(i,:)-desirepospos)>0.1
                finalvpath(i,:)=vpath(i,:);
                lf=i;
            end
        end
        finalvpath(lf+1,:)=vpath(lf+1,:);
        finalvpath
            for i=2:size(finalvpath,1)
                path_following(finalvpath(i,:));
                if i==size(finalvpath,1)
                    [o_l,o_r]=move(0,0);
                    vrep.simxSetJointTargetVelocity(clientID,lmotor,o_l,vrep.simx_opmode_oneshot);
                    vrep.simxSetJointTargetVelocity(clientID,rmotor,o_r,vrep.simx_opmode_oneshot);
                end
            end
            pause(2);
            break
        end
end
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
vrep.simxFinish(clientID);
vrep.delete(); % call the destructor!