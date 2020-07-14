clear
close all
clc
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
   clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
  
if (clientID>-1)
    disp('Connected to remote API server');
    
    %joints handles
    h=[0,0,0];
    [r, h(1)]=sim.simxGetObjectHandle(clientID,'shoulder',sim.simx_opmode_blocking);
    [r, h(2)]=sim.simxGetObjectHandle(clientID,'UpperArm',sim.simx_opmode_blocking);
    [r, h(3)]=sim.simxGetObjectHandle(clientID,'LowArm',sim.simx_opmode_blocking);
    %sim.simxSetJointTargetPosition(clientID,h(3),-pi/3,sim.simx_opmode_streaming);
    %sim.simxSetJointTargetPosition(clientID,h(2),0,sim.simx_opmode_streaming);
    %sim.simxSetJointTargetPosition(clientID,h(1),0,sim.simx_opmode_streaming);
    %sim.simxPauseCommunication(clientID,0);
    [r,j1p]=sim.simxGetJointPosition(clientID,h(1),sim.simx_opmode_blocking);
    [r,j2p]=sim.simxGetJointPosition(clientID,h(2),sim.simx_opmode_blocking);
    [r,j3p]=sim.simxGetJointPosition(clientID,h(3),sim.simx_opmode_blocking);
    [time]=sim.simxGetLastCmdTime(clientID);
    %positiontable1={j1p};
    %positiontable2={j2p};
    positiontable3=zeros(1,1);
    positiontable3(1)=j3p;
    %tic
    %time=toc;
    timetable=zeros(1,1);
    timetable(1)=time/1000;
    desiredv3=zeros(1,1);
    desiredv3(1)=1;
    error=zeros(1,1);
    error(1)=0;
    num=1;
    while j3p<pi/3
        [r]=sim.simxSetJointTargetVelocity(clientID,h(3),desiredv3(1),sim.simx_opmode_streaming);
        %time=toc;
        [r,j3p]=sim.simxGetJointPosition(clientID,h(3),sim.simx_opmode_blocking);
        [time]=sim.simxGetLastCmdTime(clientID);
        num=num+1;
        timetable(num) = time/1000;
        %[r,j2p]=sim.simxGetJointPosition(clientID,h(2),sim.simx_opmode_blocking);
        %[r,j1p]=sim.simxGetJointPosition(clientID,h(1),sim.simx_opmode_blocking);
        if j3p>=pi/3
            [r]=sim.simxSetJointTargetVelocity(clientID,h(3),0,sim.simx_opmode_streaming);
            %positiontable1(end+1) = {j1p};
            %positiontable2(end+1) = {j2p};
            positiontable3(num) = j3p;
            break
        end
        %positiontable1(end+1) = {j1p};
        %positiontable2(end+1) = {j2p};
        positiontable3(num) = j3p;
        j3v=(positiontable3(num)-positiontable3(num-1))/(timetable(num)-timetable(num-1));
        %error(num)=desiredv3(1)-j3v;
        pgain=0.1;
        dgain=0.008;
        %desiredv3(num)=pgain*error(num)+dgain*(error(num)-error(end-1))/(timetable(num)-timetable(num-1));
        
    end
    
    
    %positiontable1=cell2mat(positiontable1);
    %positiontable2=cell2mat(positiontable2);
    
    figure, hold on
    %plot(timetable, positiontable1);
    %plot(timetable, positiontable2);
    plot(timetable, positiontable3);
    title('joint position');
    xlabel('time(s)')
    ylabel('joint position[rad]')
    legend({'LowArm'})
    
    figure, hold on
    
    %plot(timetable, gradient(positiontable1)./gradient(timetable));
    %plot(timetable, gradient(positiontable2)./gradient(timetable));
    plot(timetable, gradient(positiontable3)./gradient(timetable));
    title('joint velocity');
    xlabel('time(s)')
    ylabel('joint velocity[rad/s]')
    legend({'LowArm'})
    
else
        disp('Failed connecting to remote API server');
end
    sim.delete(); % call the destructor!
    
    disp('Program ended');