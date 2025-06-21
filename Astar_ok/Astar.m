function parent=Astar()
Xo=[2 2];%���λ��
xTarget=9;%Ŀ���x����
yTarget=9;%Ŀ���y����
m_Target=[xTarget,yTarget];%Ŀ���
J=200;%ѭ����������
Xj=Xo;%j=1ѭ����ʼ���������˵���ʼ���긳��Xj
MAX_X=10;%��ͼX�߽�
MAX_Y=10;%��ͼY�߽�
MAP=2*(ones(MAX_X,MAX_Y));%�����е���ֵ����ʼ��Ϊ2������·���㡢һ��㡢�ϰ���
%ѡ����������λ��
xval=2;
yval=2;
xStart=xval;
yStart=yval;
xNode=xval;
yNode=yval;
path_cost=0;%��ʼ����Ϊ0
OPEN_COUNT=1;
MAP(xval,yval)=1;%���������λ������ֵ��ʼ��Ϊ1
MAP(1,4)=-1;%�����ϰ�����ֵ��ʼ��Ϊ-1
MAP(1,5)=-1;   
MAP(1,9)=-1;
MAP(1,10)=-1;
MAP(2,4)=-1;
MAP(2,5)=-1;
MAP(2,9)=-1;
MAP(2,10)=-1;
MAP(3,5)=-1;
MAP(3,6)=-1;
MAP(3,9)=-1;
MAP(3,10)=-1;
MAP(4,2)=-1;
MAP(4,3)=-1;
MAP(4,5)=-1;
MAP(4,6)=-1;
MAP(5,2)=-1;
MAP(5,3)=-1;
MAP(6,6)=-1;
MAP(8,2)=-1;
MAP(8,8)=-1;
MAP(9,5)=-1;

%%%%%%%%%%%%%%%��ʼ����%%%%%%%%%%%%%%%%
OPEN=[];%���ڴ�ſ���·����ʹ��ۺ���ֵ��
CLOSED=[];
j=0;
k=1;%�洢�ϰ�����
for i=1:MAX_X
    for j=1:MAX_Y
        if(MAP(i,j) == -1)
            CLOSED(k,1)=i; 
            CLOSED(k,2)=j; 
            k=k+1;
        end
    end
end
CLOSED_COUNT=size(CLOSED,1);%�����ж��ٸ��ϰ�
goal_distance=distance(xNode,yNode,xTarget,yTarget);
OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,...
    goal_distance,goal_distance);
parent(1,1)=xval;
parent(1,2)=yval;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for step=2:J%ѭ����ʼ
    Goal(step,1)=m_Target(1);
    Goal(step,2)=m_Target(2);
    exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,...
        MAX_X,MAX_Y);
    exp_count=size(exp_array,1);

 for i=1:exp_count
    %�ж��Ƿ�ֹͣѭ��
    if OPEN(OPEN_COUNT,2)==xTarget&&OPEN(OPEN_COUNT,3)==yTarget||...
            goal_distance<sqrt(2)
        OPEN(:,2)=xTarget;
        OPEN(:,3)=yTarget;%% ��ǰ����Ŀ�ĵصĻ��Ͱ�Ŀ�ĵ�д��open
    else
        %OPEN������һ��ֵ
        OPEN_COUNT=OPEN_COUNT+1;
        OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),...
        xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
    end;
 end
 best_num=2;
 L=size(OPEN,1);
 for OPEN_COUNT=2:1:L
     %�ȽϹ��ۺ�����С
     if OPEN(OPEN_COUNT,8)<OPEN(best_num,8)
         best_num=OPEN_COUNT;    
     end
 end
 OPEN_COUNT=1;%���OPEN 
 OPEN(OPEN_COUNT,:)=OPEN(best_num,:);
 %ѡ����һ��
 parent(step,1)=OPEN(best_num,2);
 parent(step,2)=OPEN(best_num,3);
 xNode=parent(step,1);
 yNode=parent(step,2);
  goal_distance=distance(xNode,yNode,xTarget,yTarget); 
end











